"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.RedisWebSocketBridge = void 0;
const http_1 = __importDefault(require("http"));
const ws_1 = __importDefault(require("ws"));
const redis_1 = require("redis");
const ioredis_1 = __importDefault(require("ioredis"));
const camera_endpoint_1 = require("./camera-endpoint");
const sim_control_endpoint_1 = require("./sim-control-endpoint");
const sim_process_manager_1 = require("./sim-process-manager");
const frame_cache_1 = require("./frame-cache");
const constants_1 = require("../rendering/constants");
class RedisWebSocketBridge {
    constructor(config) {
        this.wss = null;
        this.redisClient = null;
        this.camHttpServer = null;
        this.camRedis = null;
        this.frameStore = null;
        this.clients = new Set();
        this.subscriptions = new Map();
        // Spec 024: 仿真会话编排器(子进程管理)。
        this.simManager = null;
        this.config = config;
    }
    async start() {
        // Connect to Redis
        this.redisClient = (0, redis_1.createClient)({
            socket: {
                host: this.config.redisHost,
                port: this.config.redisPort
            },
            password: this.config.redisPassword
        });
        this.redisClient.on('error', (err) => {
            console.error('Redis client error:', err);
        });
        await this.redisClient.connect();
        console.log('Connected to Redis');
        // Create WebSocket server
        this.wss = new ws_1.default.Server({ port: this.config.wsPort });
        this.wss.on('connection', (ws) => {
            console.log('Client connected');
            this.clients.add(ws);
            // Spec 024 (FR-013): 新客户端连入时(含刷新/接入已运行的仿真)立即补推一次
            // 当前会话状态。否则前端只能靠单次 HTTP getStatus 恢复(脆弱,失败即按钮锁死),
            // 且 WS session 推送仅在状态变更时触发,刷新后无变更 → 永远收不到 running 态。
            // 修复"运行中刷新网页后暂停/关闭按钮不可点击"。
            if (this.simManager) {
                const s = this.simManager.getState();
                ws.send(JSON.stringify({ type: 'session', ...s }));
            }
            ws.on('message', (data) => {
                this.handleClientMessage(ws, data.toString());
            });
            ws.on('close', () => {
                console.log('Client disconnected');
                this.clients.delete(ws);
                this.removeClientFromSubscriptions(ws);
            });
            ws.on('error', (error) => {
                console.error('Client WebSocket error:', error);
                this.clients.delete(ws);
            });
        });
        console.log(`WebSocket server listening on port ${this.config.wsPort}`);
        // 仿真会话编排器(competition 单进程子进程管理 + 会话状态 WS 广播)。
        if (this.config.scenariosDir) {
            this.simManager = new sim_process_manager_1.SimProcessManager((0, sim_process_manager_1.createProductionDeps)({
                redisHost: this.config.redisHost,
                redisPort: this.config.redisPort,
                stopGrace: this.config.stopGrace ?? 5,
                commandChannel: constants_1.CHANNELS.commands,
                stateChannel: constants_1.CHANNELS.state,
                onStateChange: (s) => this.broadcastToAll({ type: 'session', ...s }),
            }));
            console.log(`Sim session manager enabled (scenarios: ${this.config.scenariosDir})`);
        }
        else {
            console.log('Sim session manager disabled (OPENSIM_SCENARIOS_DIR unset)');
        }
        // 022: 相机帧 HTTP 端点(ioredis 二进制安全读 hash image)。
        await this.startCameraHttp();
    }
    /** 启动相机帧 HTTP 端点 GET /cam/:uid/latest(消费 sync_camera hash)。 */
    async startCameraHttp() {
        const port = this.config.camHttpPort ?? 8081;
        this.camRedis = new ioredis_1.default({
            host: this.config.redisHost,
            port: this.config.redisPort,
            password: this.config.redisPassword,
            // 仅读,不订阅。
        });
        // 后台缓存层:KEYS+hgetBuffer 每 200ms 一次(5Hz),
        // HTTP handler 直接从内存返回(0 Redis 调用)。
        // 避免 30Hz HTTP × 800ms KEYS 把 Redis 单线程打满。
        const frameStore = new frame_cache_1.CachedFrameStore({
            keys: (p) => this.camRedis.keys(p),
            hgetBuffer: (k, f) => this.camRedis.hgetBuffer(k, f),
        });
        frameStore.start();
        this.frameStore = frameStore;
        const handler = (0, camera_endpoint_1.createCameraHandler)({
            keys: (p) => this.camRedis.keys(p),
            hgetBuffer: (k, f) => this.camRedis.hgetBuffer(k, f),
            frameStore, // 注入缓存;camera-endpoint 优先用缓存,缺省回落 readLatestFrame
        });
        const simHandler = this.simManager && this.config.scenariosDir
            ? (0, sim_control_endpoint_1.createSimControlHandler)({
                manager: this.simManager,
                scenariosDir: this.config.scenariosDir,
                pythonBin: this.config.pythonBin ?? 'python',
                // Spec 028: 透传渲染器编排配置,使 bridge spawn/监控 UE。
                // 缺省(renderCtlBinary undefined)→ 渲染器子系统休眠(仿真照跑)。
                renderCtlBinary: this.config.renderCtlBinary,
                renderersDir: this.config.renderersDir,
                userAlgorithmsDir: this.config.userAlgorithmsDir,
            })
            : null;
        this.camHttpServer = http_1.default.createServer((req, res) => {
            const url = req.url || '';
            const dispatch = url.startsWith('/api/') && simHandler ? simHandler : handler;
            dispatch(req, res).catch((err) => {
                console.error('http handler error:', err);
                if (!res.headersSent) {
                    res.writeHead(500, { 'Content-Type': 'application/json' });
                    res.end(JSON.stringify({ error: 'internal' }));
                }
            });
        });
        await new Promise((resolve) => this.camHttpServer.listen(port, resolve));
        console.log(`Camera frame HTTP endpoint listening on port ${port} (GET /cam/:uid/latest)`);
    }
    async stop() {
        if (this.simManager) {
            await this.simManager.stop().catch((err) => {
                console.warn('failed to stop simulation session:', err);
            });
            await this.simManager.unsubscribe().catch((err) => {
                console.warn('failed to unsubscribe from state channel:', err);
            });
        }
        if (this.camHttpServer) {
            await new Promise((resolve) => this.camHttpServer.close(() => resolve()));
            this.camHttpServer = null;
        }
        if (this.frameStore) {
            this.frameStore.stop();
            this.frameStore = null;
        }
        if (this.camRedis) {
            this.camRedis.disconnect();
            this.camRedis = null;
        }
        if (this.wss) {
            this.wss.close();
        }
        if (this.redisClient) {
            await this.redisClient.disconnect();
        }
    }
    async handleClientMessage(ws, data) {
        try {
            const message = JSON.parse(data);
            switch (message.type) {
                case 'subscribe':
                    await this.handleSubscribe(ws, message.channels);
                    break;
                case 'unsubscribe':
                    await this.handleUnsubscribe(ws, message.channels);
                    break;
                case 'publish':
                    await this.handlePublish(message.channel, message.message);
                    break;
                default:
                    ws.send(JSON.stringify({ type: 'error', error: 'Unknown message type' }));
            }
        }
        catch (error) {
            ws.send(JSON.stringify({ type: 'error', error: 'Invalid message format' }));
        }
    }
    async handleSubscribe(ws, channels) {
        if (!this.redisClient)
            return;
        for (const channel of channels) {
            if (!this.subscriptions.has(channel)) {
                this.subscriptions.set(channel, new Set());
                // Subscribe to Redis channel
                await this.redisClient.subscribe(channel, (message) => {
                    this.broadcastToSubscribers(channel, message);
                });
            }
            this.subscriptions.get(channel).add(ws);
        }
        ws.send(JSON.stringify({ type: 'subscribe', channels }));
    }
    async handleUnsubscribe(ws, channels) {
        for (const channel of channels) {
            const subscribers = this.subscriptions.get(channel);
            if (subscribers) {
                subscribers.delete(ws);
                if (subscribers.size === 0) {
                    this.subscriptions.delete(channel);
                    // Unsubscribe from Redis channel if no more subscribers
                    if (this.redisClient) {
                        await this.redisClient.unsubscribe(channel);
                    }
                }
            }
        }
        ws.send(JSON.stringify({ type: 'unsubscribe', channels }));
    }
    async handlePublish(channel, message) {
        if (!this.redisClient)
            return;
        await this.redisClient.publish(channel, message);
    }
    broadcastToSubscribers(channel, message) {
        const subscribers = this.subscriptions.get(channel);
        if (!subscribers)
            return;
        const payload = JSON.stringify({
            type: 'message',
            channel,
            message
        });
        for (const ws of subscribers) {
            if (ws.readyState === ws_1.default.OPEN) {
                ws.send(payload);
            }
        }
    }
    /** Spec 024 (T047): 向所有已连接 WS 客户端主动广播(会话状态推送,不经 Redis)。 */
    broadcastToAll(payload) {
        const data = JSON.stringify(payload);
        for (const ws of this.clients) {
            if (ws.readyState === ws_1.default.OPEN)
                ws.send(data);
        }
    }
    removeClientFromSubscriptions(ws) {
        for (const [channel, subscribers] of this.subscriptions) {
            subscribers.delete(ws);
            if (subscribers.size === 0) {
                this.subscriptions.delete(channel);
            }
        }
    }
}
exports.RedisWebSocketBridge = RedisWebSocketBridge;
