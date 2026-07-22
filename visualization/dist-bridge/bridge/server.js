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
const sim_control_endpoint_1 = require("./sim-control-endpoint");
const sim_process_manager_1 = require("./sim-process-manager");
const frame_cache_1 = require("./frame-cache");
const camera_ws_server_1 = require("./camera-ws-server");
const constants_1 = require("../rendering/constants");
class RedisWebSocketBridge {
    constructor(config) {
        this.wss = null;
        this.redisClient = null;
        this.camHttpServer = null;
        this.camRedis = null;
        this.frameStore = null;
        this.cameraWsServer = null;
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
        // 022: 相机帧服务(WS 推送 + HTTP /api/ 仿真控制)。
        await this.startCameraService();
    }
    /**
     * 启动相机帧服务:
     *   - camHttpServer(:8081):仅服务 /api/ 仿真控制端点(原 camera HTTP 端点
     *     已迁移到 WS 推送,见下方 cameraWsServer)。
     *   - frameStore:后台游标追帧,WS 推送的数据源。
     *   - cameraWsServer(:8082):相机帧 WS 推送,frameStore 新帧 → broadcast。
     */
    async startCameraService() {
        const port = this.config.camHttpPort ?? 8081;
        this.camRedis = new ioredis_1.default({
            host: this.config.redisHost,
            port: this.config.redisPort,
            password: this.config.redisPassword,
            // 仅读,不订阅。
        });
        // 后台缓存层:按 uid 游标追帧(优先 N+1 hget),每 ~33ms 刷新一次,
        // 与 UE 30Hz 写帧对齐。WS 推送 server 据此 broadcast 给订阅者。
        const frameStore = new frame_cache_1.CachedFrameStore({
            keys: (p) => this.camRedis.keys(p),
            hgetBuffer: (k, f) => this.camRedis.hgetBuffer(k, f),
        }, { refreshMs: 33 });
        frameStore.start();
        this.frameStore = frameStore;
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
            // 仅服务 /api/ 仿真控制;相机帧已走 WS 推送(见下方 cameraWsServer)。
            if (url.startsWith('/api/') && simHandler) {
                simHandler(req, res).catch((err) => {
                    console.error('http handler error:', err);
                    if (!res.headersSent) {
                        res.writeHead(500, { 'Content-Type': 'application/json' });
                        res.end(JSON.stringify({ error: 'internal' }));
                    }
                });
                return;
            }
            res.writeHead(404, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ error: 'not_found' }));
        });
        await new Promise((resolve) => this.camHttpServer.listen(port, resolve));
        console.log(`Camera HTTP (sim-control /api/) listening on port ${port}`);
        // 022: 相机帧 WebSocket 推送 server(替代 HTTP 拉取,支持多路 + 10 路目标)。
        // frameStore 刷新出新帧时,WS server 主动 broadcast 给订阅该 uid 的客户端。
        // 前端一条长连接,无 HTTP 短连接的连接池饱和/700ms 尖刺问题。
        const wsPort = this.config.camWsPort ?? 8082;
        this.cameraWsServer = new camera_ws_server_1.CameraWsServer({
            port: wsPort,
            frameStore,
            redis: {
                keys: (p) => this.camRedis.keys(p),
                hgetBuffer: (k, f) => this.camRedis.hgetBuffer(k, f),
            },
        });
        await this.cameraWsServer.start();
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
        if (this.cameraWsServer) {
            await this.cameraWsServer.stop();
            this.cameraWsServer = null;
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
