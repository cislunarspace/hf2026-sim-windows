export interface BridgeConfig {
    wsPort: number;
    redisHost: string;
    redisPort: number;
    redisPassword?: string;
    /** 相机帧 HTTP 端点端口(默认 8081)。 */
    camHttpPort?: number;
    scenariosDir?: string;
    userAlgorithmsDir?: string;
    pythonBin?: string;
    stopGrace?: number;
    renderCtlBinary?: string;
    renderersDir?: string;
}
export declare class RedisWebSocketBridge {
    private wss;
    private redisClient;
    private camHttpServer;
    private camRedis;
    private frameStore;
    private clients;
    private subscriptions;
    private config;
    private simManager;
    constructor(config: BridgeConfig);
    start(): Promise<void>;
    /** 启动相机帧 HTTP 端点 GET /cam/:uid/latest(消费 sync_camera hash)。 */
    private startCameraHttp;
    stop(): Promise<void>;
    private handleClientMessage;
    private handleSubscribe;
    private handleUnsubscribe;
    private handlePublish;
    private broadcastToSubscribers;
    /** Spec 024 (T047): 向所有已连接 WS 客户端主动广播(会话状态推送,不经 Redis)。 */
    private broadcastToAll;
    private removeClientFromSubscriptions;
}
//# sourceMappingURL=server.d.ts.map