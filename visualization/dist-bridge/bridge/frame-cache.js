"use strict";
// 022 UAV 相机帧缓存层。
//
// 背景:sync_camera:{uid}:frame:{n} hash 没有 latest 指针,消费端原本用
// KEYS + JS loop 找最大帧号。但 KEYS 在 Redis 单线程下是 O(N) 阻塞命令,
// 每帧 ~70 个 hash key 时单次 KEYS 实测 800-900ms,严重阻塞 sim 写状态
// (slowlog 显示 sim_time 停在 0)。前端 30Hz 拉帧 = 30 次 KEYS/s,Redis
// 永远在阻塞 → 画面卡顿。
//
// 修复:后台 refresh 线程定期(默认 200ms = 5Hz)扫描所有 sync_camera:* 帧
// 并读最新帧到内存;HTTP handler 直接返回缓存,0 阻塞 Redis。
// 这样 KEYS 频率从 ~30Hz 降到 5Hz,且不与 HTTP 请求绑定(突发流量也不会
// 打爆 Redis)。
//
// 内存占用:每帧 ~1.7MB PNG × N 个 uid;N 通常 ≤ 4(前端 PiP 同时最多 4 窗)。
Object.defineProperty(exports, "__esModule", { value: true });
exports.CachedFrameStore = void 0;
exports.getCachedFrame = getCachedFrame;
const frame_reader_1 = require("./frame-reader");
/**
 * 后台轮询 sync_camera 帧,缓存每个 uid 的最新帧。
 *
 * 用法:
 *   const store = new CachedFrameStore(redis);
 *   await store.start();                  // 启动后台扫描
 *   const frame = store.get(uid);         // 同步返回缓存(无 Redis 调用)
 *   store.stop();                          // 停止扫描
 */
class CachedFrameStore {
    constructor(redis, opts = {}) {
        this.cache = new Map();
        this.timer = null;
        this.refreshing = false;
        this.redis = redis;
        this.refreshMs = opts.refreshMs ?? 500;
        this._setTimeout = opts.setTimeout ?? setTimeout;
        this._clearTimeout = opts.clearTimeout ?? clearTimeout;
        this.log = opts.log ?? (() => { });
    }
    /** 启动后台扫描。立即触发一次,之后按 refreshMs 周期触发。 */
    start() {
        if (this.timer)
            return;
        void this.refresh();
    }
    /** 停止后台扫描。 */
    stop() {
        if (this.timer) {
            this._clearTimeout(this.timer);
            this.timer = null;
        }
    }
    /**
     * 同步返回指定 uid 的最新帧缓存。
     * @returns 缓存帧;uid 不在缓存中时返回 null(调用方应回退到 readLatestFrame)。
     */
    get(uid) {
        return this.cache.get(uid);
    }
    /** 标记某 uid 已停流(让 HTTP 返回 no_stream 而不是 stale 缓存)。 */
    invalidate(uid) {
        this.cache.delete(uid);
    }
    async refresh() {
        if (this.refreshing) {
            // 上一次还没完成(罕见,Redis 阻塞时可能发生);跳过本轮,等下一周期。
            this.scheduleNext();
            return;
        }
        this.refreshing = true;
        try {
            // 1. 扫描所有 sync_camera:*:frame:* key,提取 uid 集合 + 各 uid 的最大帧号。
            const keys = await this.redis.keys('sync_camera:*:frame:*');
            const uidToMaxKey = new Map();
            for (const key of keys) {
                const m = key.match(/^sync_camera:([^:]+):frame:(\d+)$/);
                if (!m || m[1] === undefined || m[2] === undefined)
                    continue;
                const uid = m[1];
                const frameNo = parseInt(m[2], 10);
                const prev = uidToMaxKey.get(uid);
                if (!prev || frameNo > prev.frameNo) {
                    uidToMaxKey.set(uid, { key, frameNo });
                }
            }
            // 2. 对每个 uid:若 maxKey 帧号 > 缓存帧号,读 image 刷新缓存。
            //    (帧号未变 = UE 还没写新帧,保留旧缓存避免重复 hgetBuffer 1.7MB)
            for (const [uid, { key, frameNo }] of uidToMaxKey) {
                const cached = this.cache.get(uid);
                if (cached && cached.frameNo >= frameNo)
                    continue; // 缓存已是最新的
                // 复用 readLatestFrame 的逻辑(它内部再 KEYS+hgetBuffer,但 keys 已知,
                // 这里直接 hgetBuffer 更省)。
                const image = await this.redis.hgetBuffer(key, 'image');
                if (!image)
                    continue;
                const simBuf = await this.redis.hgetBuffer(key, 'sim_time');
                const simTime = simBuf ? parseFloat(simBuf.toString('utf8')) : NaN;
                this.cache.set(uid, { frameNo, simTime, image });
            }
            // 3. 清理已停流的 uid(扫描结果里没有,但缓存里有的)。
            for (const uid of this.cache.keys()) {
                if (!uidToMaxKey.has(uid)) {
                    this.cache.delete(uid);
                }
            }
        }
        catch (e) {
            this.log(`[FrameCache] refresh error: ${e.message}`);
        }
        finally {
            this.refreshing = false;
            this.scheduleNext();
        }
    }
    scheduleNext() {
        this.timer = this._setTimeout(() => void this.refresh(), this.refreshMs);
    }
}
exports.CachedFrameStore = CachedFrameStore;
/**
 * 创建一个 camera HTTP handler,优先从 CachedFrameStore 同步返回缓存;
 * 缓存未命中(uid 没扫描到)时回退到 readLatestFrame(保证首帧延迟可接受)。
 */
function getCachedFrame(store, redis, uid) {
    const cached = store.get(uid);
    if (cached !== undefined) {
        return Promise.resolve(cached);
    }
    // uid 不在缓存中(可能是首帧或新开 PiP 窗口):同步阻塞读一次,
    // 后台 refresh 会接管后续。这是 KEYS 的兜底路径,频率应该很低。
    return (0, frame_reader_1.readLatestFrame)(redis, uid);
}
