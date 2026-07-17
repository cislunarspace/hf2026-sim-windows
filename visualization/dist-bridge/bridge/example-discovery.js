"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
exports.HIDDEN_EXAMPLE_IDS = void 0;
exports.validateManifest = validateManifest;
exports.discoverExamples = discoverExamples;
// Spec 024 (T005): 示例发现 —— 扫描 examples/ 下含 run.py + manifest.json 的
// 目录,按 contracts/example-manifest.md 校验 manifest,排除 _common 等。
// 纯逻辑(validateManifest)与磁盘扫描(discoverExamples)分离,便于单测。
const fs_1 = require("fs");
const path = __importStar(require("path"));
const ID_RE = /^[a-z][a-z0-9_]*$/;
const RUNNER_RE = /^examples\.[a-z_][a-z0-9_.]*\.run$/;
/**
 * 在面板/API 中隐藏的 example id 集合(目录仍保留,仅前端不列举)。
 * 用途:退役/暂不公开的 example 不再出现在 UI 选择里,但磁盘上的 run.py /
 * manifest.json / config/ 保留以便回退或归档参考。
 * 如果以后某个 example 需要恢复,从此集合中移除即可。
 */
exports.HIDDEN_EXAMPLE_IDS = new Set([
    'uav_track_road_target',
]);
/**
 * 校验单个 manifest 对象(不涉磁盘)。纯函数。
 * @param dirName 若给出,要求 manifest.id === dirName(枚举时传入)。
 * @returns 合法则返回规整后的 manifest,否则 null。
 */
function validateManifest(raw, dirName) {
    if (!raw || typeof raw !== 'object')
        return null;
    const m = raw;
    const { id, name, description, scenario, runner_module, default_duration } = m;
    if (typeof id !== 'string' || !ID_RE.test(id))
        return null;
    if (dirName !== undefined && id !== dirName)
        return null; // id 必须等于目录名
    if (typeof name !== 'string' || name.length === 0)
        return null;
    if (typeof description !== 'string' || description.length === 0)
        return null;
    if (typeof scenario !== 'string' || scenario.length === 0)
        return null;
    if (typeof runner_module !== 'string' || !RUNNER_RE.test(runner_module))
        return null;
    if (default_duration !== undefined && typeof default_duration !== 'number')
        return null;
    return {
        id,
        name,
        description,
        scenario,
        runner_module,
        default_duration: typeof default_duration === 'number' ? default_duration : undefined,
    };
}
/**
 * 扫描 examplesDir,返回有效示例(含 run.py + 合法 manifest,排除下划线前缀目录)。
 * 缺 scenario 文件的示例仍列出,但 available=false。
 */
async function discoverExamples(examplesDir) {
    const entries = await fs_1.promises.readdir(examplesDir, { withFileTypes: true });
    const examples = [];
    for (const entry of entries) {
        if (!entry.isDirectory())
            continue;
        if (entry.name.startsWith('_'))
            continue; // 排除 _common 等
        if (exports.HIDDEN_EXAMPLE_IDS.has(entry.name))
            continue; // 退役/暂不公开的 example
        const dir = path.join(examplesDir, entry.name);
        const runPy = path.join(dir, 'run.py');
        const manifestPath = path.join(dir, 'manifest.json');
        // 必须含 run.py(否则非示例)。
        try {
            await fs_1.promises.access(runPy);
        }
        catch {
            continue;
        }
        // 必须含合法 manifest.json。
        let raw;
        try {
            raw = JSON.parse(await fs_1.promises.readFile(manifestPath, 'utf8'));
        }
        catch {
            continue;
        }
        const manifest = validateManifest(raw, entry.name);
        if (!manifest)
            continue;
        // scenario 文件存在性(相对示例目录)。
        const scenarioAbs = path.join(dir, manifest.scenario);
        let available = true;
        try {
            await fs_1.promises.access(scenarioAbs);
        }
        catch {
            available = false;
        }
        examples.push({
            id: manifest.id,
            name: manifest.name,
            description: manifest.description,
            scenario: manifest.scenario,
            runnerModule: manifest.runner_module,
            defaultDuration: manifest.default_duration,
            available,
        });
    }
    examples.sort((a, b) => a.id.localeCompare(b.id));
    return examples;
}
