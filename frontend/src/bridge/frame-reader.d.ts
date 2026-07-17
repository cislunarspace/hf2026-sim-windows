/** 消费端所需的最小 redis 能力(ioredis 子集,可 mock)。 */
export interface CameraFrameRedis {
    /** 匹配 key 列表(本机帧数百级,v1 用 KEYS;SCAN 为后续优化)。 */
    keys(pattern: string): Promise<string[]>;
    /** 取 hash 字段的二进制值(二进制安全)。 */
    hgetBuffer(key: string, field: string): Promise<Buffer | null>;
}
/** 定位到的最新帧。 */
export interface LatestCameraFrame {
    /** 帧号(来自 key 名,单调递增)。 */
    frameNo: number;
    /** 生成时间戳(仿真时刻,一致性核验锚点)。 */
    simTime: number;
    /** JPEG 二进制。 */
    image: Buffer;
}
/**
 * 定位并读取指定 UAV 的最新相机帧。
 * @returns 最新帧;无帧/缺 image 字段时返回 null。
 */
export declare function readLatestFrame(redis: CameraFrameRedis, uid: string): Promise<LatestCameraFrame | null>;
//# sourceMappingURL=frame-reader.d.ts.map