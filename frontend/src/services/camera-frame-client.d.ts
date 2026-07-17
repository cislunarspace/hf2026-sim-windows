export interface CameraFrame {
    /** 帧号(单调递增,去重依据)。 */
    frameNo: number;
    /** 生成时间戳(仿真时刻,一致性核验锚点)。 */
    simTime: number;
    /** 已创建的 Blob ObjectURL(供 <img>.src;用完需 revoke)。 */
    blobUrl: string;
}
/** 去重/追帧决策(纯函数,便于单测)。仅当帧号严格大于已显示帧号才显示。 */
export declare function shouldDisplayFrame(incomingFrameNo: number, displayedFrameNo: number): boolean;
export interface CameraClientOptions {
    /** 构造拉取 URL。 */
    endpoint: (uid: string) => string;
    /** 收到新帧(或 null=无流/停流)时回调。 */
    onFrame: (frame: CameraFrame | null) => void;
    /** 拉取周期(毫秒),默认 ~30Hz。 */
    intervalMs?: number;
    /** 注入 fetch(测试用)。 */
    fetchImpl?: typeof fetch;
}
export declare class CameraFrameClient {
    private readonly endpoint;
    private readonly onFrame;
    private readonly intervalMs;
    private readonly fetchImpl;
    private timer;
    private abort;
    private inFlight;
    private uid;
    private displayedFrameNo;
    private currentUrl;
    constructor(opts: CameraClientOptions);
    /** 开始为指定 UAV 拉取(会先停止旧流)。 */
    start(uid: string): void;
    /** 停止拉取并释放当前帧资源。 */
    stop(): void;
    private tick;
    private releaseUrl;
}
//# sourceMappingURL=camera-frame-client.d.ts.map