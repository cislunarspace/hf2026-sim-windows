import { type CameraFrameRedis, type LatestCameraFrame } from './frame-reader';
/** uid → 最新帧缓存(null 表示该 uid 暂无帧)。 */
export type FrameCacheMap = Map<string, LatestCameraFrame | null>;
export interface FrameCacheOptions {
    /** 后台扫描周期(毫秒),默认 500ms(2Hz)。
     *  UE 实测写帧 7-15fps,2Hz 后台扫描完全够用;
     *  Redis 单线程下 KEYS(实测 400-800ms)+ UE 写 1.7MB HSET 共享同一进程,
     *  KEYS 频率越低 Redis 越空闲,UE 写帧越快。 */
    refreshMs?: number;
    /** 注入的 setTimeout/NSDate(测试用)。 */
    setTimeout?: typeof setTimeout;
    /** 注入的 clearTimeout(测试用)。 */
    clearTimeout?: typeof clearTimeout;
    /** 日志(测试用)。 */
    log?: (msg: string) => void;
}
/**
 * 后台轮询 sync_camera 帧,缓存每个 uid 的最新帧。
 *
 * 用法:
 *   const store = new CachedFrameStore(redis);
 *   await store.start();                  // 启动后台扫描
 *   const frame = store.get(uid);         // 同步返回缓存(无 Redis 调用)
 *   store.stop();                          // 停止扫描
 */
export declare class CachedFrameStore {
    private readonly redis;
    private readonly refreshMs;
    private readonly _setTimeout;
    private readonly _clearTimeout;
    private readonly log;
    private cache;
    private timer;
    private refreshing;
    constructor(redis: CameraFrameRedis, opts?: FrameCacheOptions);
    /** 启动后台扫描。立即触发一次,之后按 refreshMs 周期触发。 */
    start(): void;
    /** 停止后台扫描。 */
    stop(): void;
    /**
     * 同步返回指定 uid 的最新帧缓存。
     * @returns 缓存帧;uid 不在缓存中时返回 null(调用方应回退到 readLatestFrame)。
     */
    get(uid: string): LatestCameraFrame | null | undefined;
    /** 标记某 uid 已停流(让 HTTP 返回 no_stream 而不是 stale 缓存)。 */
    invalidate(uid: string): void;
    private refresh;
    private scheduleNext;
}
/**
 * 创建一个 camera HTTP handler,优先从 CachedFrameStore 同步返回缓存;
 * 缓存未命中(uid 没扫描到)时回退到 readLatestFrame(保证首帧延迟可接受)。
 */
export declare function getCachedFrame(store: CachedFrameStore, redis: CameraFrameRedis, uid: string): Promise<LatestCameraFrame | null>;
//# sourceMappingURL=frame-cache.d.ts.map