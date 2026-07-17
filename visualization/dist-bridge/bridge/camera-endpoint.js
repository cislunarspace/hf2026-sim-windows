"use strict";
// 022 UAV 相机视角实时视频流 — bridge HTTP 帧投递端点。
//
// 暴露 GET /cam/:uav_id/latest:从本机 Redis 读最新帧(hash sync_camera),
// 返回 JPEG + 头(X-Frame-No / X-Sim-Ts)。前端 30Hz 拉取。
// CORS 开放(前端 dev server :3000 跨域访问 bridge :8081)。
// 详见 specs/022-uav-camera-feed/contracts/frame-delivery.md
//
// 性能:若注入 frameStore,优先从缓存同步返回(0 Redis 调用);
// 缺省回退到 readLatestFrame(每次请求 KEYS,会阻塞 Redis,仅用于测试)。
Object.defineProperty(exports, "__esModule", { value: true });
exports.createCameraHandler = createCameraHandler;
const frame_reader_1 = require("./frame-reader");
const ROUTE_RE = /^\/cam\/([^/]+)\/latest$/;
const CORS = {
    'Access-Control-Allow-Origin': '*',
    'Access-Control-Allow-Methods': 'GET, OPTIONS',
    'Access-Control-Allow-Headers': 'Content-Type, X-Frame-No, X-Sim-Ts',
    'Access-Control-Expose-Headers': 'X-Frame-No, X-Sim-Ts',
};
/** 构造 HTTP 请求处理函数(注入 redis,便于测试)。 */
function createCameraHandler(redis) {
    return async (req, res) => {
        // CORS 预检
        if (req.method === 'OPTIONS') {
            res.writeHead(204, CORS);
            res.end();
            return;
        }
        const m = req.url?.match(ROUTE_RE);
        const uid = m?.[1];
        if (!uid) {
            res.writeHead(404, { ...CORS, 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ error: 'not_found' }));
            return;
        }
        const decodedUid = decodeURIComponent(uid);
        try {
            // 优先用缓存(同步返回,不阻塞 Redis);缺省回退 readLatestFrame。
            let frame;
            if (redis.frameStore) {
                const cached = redis.frameStore.get(decodedUid);
                frame = cached === undefined ? await (0, frame_reader_1.readLatestFrame)(redis, decodedUid) : cached;
            }
            else {
                frame = await (0, frame_reader_1.readLatestFrame)(redis, decodedUid);
            }
            if (!frame) {
                res.writeHead(404, { ...CORS, 'Content-Type': 'application/json' });
                res.end(JSON.stringify({ error: 'no_stream', uav_id: decodedUid }));
                return;
            }
            res.writeHead(200, {
                ...CORS,
                'Content-Type': 'image/jpeg',
                'X-Frame-No': String(frame.frameNo),
                'X-Sim-Ts': String(frame.simTime),
                'Cache-Control': 'no-store',
            });
            res.end(frame.image);
        }
        catch {
            res.writeHead(500, { ...CORS, 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ error: 'internal' }));
        }
    };
}
