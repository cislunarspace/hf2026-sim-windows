import type { CachedFrameStore } from './frame-cache';
import type { CameraFrameRedis, LatestCameraFrame } from './frame-reader';
/** 固定 header 长度(字节)。 */
declare const HEADER_LEN = 16;
export interface CameraWsServerOptions {
    /** 监听端口。 */
    port: number;
    /** 帧缓存(推送数据源 + subscribe 时读已有缓存帧)。 */
    frameStore: CachedFrameStore;
    /** Redis(subscribe 时若 frameStore 无缓存,回退 readLatestFrame 读首帧)。 */
    redis: CameraFrameRedis;
    /** 日志(默认 console.log)。 */
    log?: (msg: string) => void;
}
/**
 * 相机帧 WS 推送 server。
 *
 * 生命周期:由 server.ts 在 startCameraService 中创建,start() 启动,
 * stop() 关闭。推送钩子(onFrameUpdate)在 frameStore 刷新新帧时被调用。
 */
export declare class CameraWsServer {
    private readonly port;
    private readonly frameStore;
    private readonly redis;
    private readonly log;
    private wss;
    /** uid → 订阅它的客户端集合。 */
    private subscriptions;
    /** ws → 它订阅的 uid 集合(反向索引,断开时 O(1) 清理)。 */
    private clientUids;
    /** 推送钩子解绑函数(start 时挂到 frameStore)。 */
    private unbindHook;
    /** 诊断:统计推送次数/字节数(日志节流输出)。 */
    private pushedFrames;
    private pushedBytes;
    private lastStatsAt;
    constructor(opts: CameraWsServerOptions);
    /** 启动 WS server 并挂上 frameStore 推送钩子。 */
    start(): Promise<void>;
    /** 关闭 WS server + 解绑推送钩子。 */
    stop(): Promise<void>;
    private handleConnection;
    private handleTextMessage;
    private handleDisconnect;
    private addSubscription;
    private removeSubscription;
    /** 把一帧推给订阅该 uid 的所有 OPEN 客户端。 */
    private pushFrame;
    /** subscribe 后立即推已有缓存帧(若 frameStore 有)。 */
    private pushInitialFrame;
    /** 节流统计日志:每 5 秒打一次推送量。 */
    private maybeLogStats;
    private sendControl;
}
/**
 * 编码一帧为 WS binary message。
 * 格式见文件头注释(16 字节 header + uid + image)。
 * 导出供单测。
 */
export declare function encodeFrameMessage(frame: LatestCameraFrame, uid: string): Buffer;
export { HEADER_LEN };
//# sourceMappingURL=camera-ws-server.d.ts.map