import type { IncomingMessage, ServerResponse } from 'http';
import { type CameraFrameRedis } from './frame-reader';
import type { CachedFrameStore } from './frame-cache';
export interface CameraHandlerRedis extends CameraFrameRedis {
    /** 可选的缓存层。注入则 HTTP 请求优先从缓存返回,避免阻塞 Redis。 */
    frameStore?: CachedFrameStore;
}
/** 构造 HTTP 请求处理函数(注入 redis,便于测试)。 */
export declare function createCameraHandler(redis: CameraHandlerRedis): (req: IncomingMessage, res: ServerResponse) => Promise<void>;
//# sourceMappingURL=camera-endpoint.d.ts.map