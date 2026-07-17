import * as THREE from 'three';
import { GimbalState } from '../core/wgs84-projection';
/**
 * Gimbal direction ray + FOV cone.
 *
 * Both are sized so their tip lands on the terrain at the UAV's ground
 * projection. The ray length is computed each update from the UAV's
 * altitude above ground and the gimbal tilt angle:
 *
 *     L = altitudeAboveGround / sin(|tilt|)
 *
 * (tilt is the depression angle from horizontal; at -90 deg the ray
 * points straight down and L == altitudeAboveGround; at shallow tilts L
 * grows but is capped so the cone doesn't shoot past the horizon). This
 * fixes the original bug where the ray had a fixed 330-unit length and
 * the cone a fixed 30-unit length, so when a UAV flew high the ray
 * stopped in mid-air well above the ground.
 *
 * Geometry note: the indicator is attached as a child of the UAV model
 * and lives in the UAV body frame where local -Z is "forward" (nose).
 * The model group is rotated by pan (Y) then tilt (X) so local -Z
 * points along the gimbal boresight. WGS84Projection maps 1 metre of
 * altitude to 1 Three.js unit (Y), so the altitude delta in metres is
 * directly the ray length in world units when looking straight down.
 */
export declare class GimbalIndicator {
    private model;
    private directionLine;
    private fovCone;
    private static readonly kMinTiltDeg;
    private static readonly kMaxRayM;
    private static readonly kMinRayM;
    constructor();
    /**
     * @param gimbalState  pan/tilt/fov from the kernel.
     * @param uavAltitudeM the UAV's current altitude (metres).
     * @param groundAltitudeM  terrain elevation under the UAV (metres). 0
     *        when no heightmap is loaded — in that case the ray falls back
     *        to a length proportional to the UAV altitude.
     */
    update(gimbalState: GimbalState, uavAltitudeM?: number, groundAltitudeM?: number): void;
    setFovVisible(visible: boolean): void;
    getModel(): THREE.Group;
}
//# sourceMappingURL=gimbal-indicator.d.ts.map