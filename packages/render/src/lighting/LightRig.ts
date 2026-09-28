import { Quat, type RigidBody, Vec3 } from '@thunder/physics';
import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { type LightingPreset, type LightingPresetName, lightingPresets } from './presets';
import { directionFromAngles, snapShadowCenter } from './shadow';

export type ShadowQuality = 'off' | 'low' | 'medium' | 'high';

const SHADOW_MAP_SIZE: Record<Exclude<ShadowQuality, 'off'>, number> = {
  low: 1024,
  medium: 2048,
  high: 4096,
};

export interface SunShadowOptions {
  /** 阴影覆盖范围的半宽（米），以跟随点为中心，默认 25 */
  halfExtent?: number;
  /** 光源到跟随点的距离（米），决定阴影相机的近 / 远平面，默认 60 */
  distance?: number;
  bias?: number;
  normalBias?: number;
  /** 阴影边缘柔和度（PCF 采样半径，单位纹素），默认 2 */
  softness?: number;
}

export interface PointLightOptions {
  color?: THREE.ColorRepresentation;
  intensity?: number;
  /** 照射距离（0 表示无限） */
  distance?: number;
  decay?: number;
  position?: THREE.Vector3Like;
  /** 点光源阴影需要渲染 6 次，数量多时开销很大 */
  castShadow?: boolean;
  shadowMapSize?: number;
}

export interface SpotLightOptions extends PointLightOptions {
  /** 目标点（世界坐标） */
  target?: THREE.Vector3Like;
  /** 光锥半角（弧度），默认 π/6 */
  angle?: number;
  /** 边缘柔和度 0..1，默认 0.3 */
  penumbra?: number;
}

export interface AttachOptions {
  /** 相对刚体原点的局部偏移 */
  offset?: THREE.Vector3Like;
  /** 聚光灯在刚体局部坐标系中的照射方向，默认 (0, -1, 0) */
  direction?: THREE.Vector3Like;
}

interface Attachment {
  body: RigidBody;
  offset: Vec3;
  direction: Vec3;
}

const tmpP = new Vec3();
const tmpQ = new Quat();
const tmpV = new Vec3();
const tmpFocus = new THREE.Vector3();

/**
 * 光源系统：
 * - 主光（平行光，太阳/月亮）+ 半球天光 + 环境光 + 可选环境贴图（IBL）
 * - 任意数量的点光源 / 聚光灯，可绑定到物理刚体上随之移动
 * - 主光阴影以跟随点为中心并做纹素对齐，大场景中阴影不会被截断、移动时不闪烁
 * - 光照预设：室内、白天、黄昏、夜晚、阴天
 */
export class LightRig {
  /** 所有光源都挂在这个组下，把它加入场景即可 */
  readonly group = new THREE.Group();
  readonly sun: THREE.DirectionalLight;
  readonly hemisphere: THREE.HemisphereLight;
  readonly ambient: THREE.AmbientLight;
  /** 指向主光的单位方向 */
  readonly sunDirection = new THREE.Vector3(0, 1, 0);
  readonly shadow: Required<SunShadowOptions>;
  private readonly extras = new Set<THREE.PointLight | THREE.SpotLight>();
  private readonly attachments = new Map<THREE.Light, Attachment>();
  private readonly shadowCenter = new THREE.Vector3();
  private environmentMap: THREE.Texture | null = null;
  private quality: ShadowQuality = 'medium';
  /** 当前预设是否让主光投射阴影（与阴影质量共同决定 castShadow） */
  private sunShadowEnabled = true;
  private presetName: LightingPresetName | null = null;

  constructor(shadow: SunShadowOptions = {}) {
    this.group.name = 'LightRig';
    this.shadow = {
      halfExtent: shadow.halfExtent ?? 25,
      distance: shadow.distance ?? 60,
      bias: shadow.bias ?? -0.0004,
      normalBias: shadow.normalBias ?? 0.02,
      softness: shadow.softness ?? 2,
    };
    this.sun = new THREE.DirectionalLight(0xffffff, 1.5);
    this.sun.name = 'Sun';
    this.sun.castShadow = true;
    this.sun.shadow.bias = this.shadow.bias;
    this.sun.shadow.normalBias = this.shadow.normalBias;
    this.sun.shadow.radius = this.shadow.softness;
    this.hemisphere = new THREE.HemisphereLight(0xffffff, 0x444444, 0.5);
    this.ambient = new THREE.AmbientLight(0xffffff, 0);
    this.group.add(this.sun, this.sun.target, this.hemisphere, this.ambient);
    this.setShadowQuality('medium');
    this.setSunAngles(40, 55);
  }

  /** 当前预设名称（手动修改过光源时仍返回最后应用的预设） */
  get preset(): LightingPresetName | null {
    return this.presetName;
  }

  /** 设置主光方向（方位角、仰角，单位度） */
  setSunAngles(azimuthDeg: number, elevationDeg: number): void {
    directionFromAngles(azimuthDeg, elevationDeg, this.sunDirection);
    this.updateSunTransform();
  }

  /** 阴影质量：off 关闭主光阴影，low/medium/high 对应 1024/2048/4096 贴图 */
  setShadowQuality(quality: ShadowQuality): void {
    this.quality = quality;
    this.sun.castShadow = quality !== 'off' && this.sunShadowEnabled;
    if (quality === 'off') return;
    const size = SHADOW_MAP_SIZE[quality];
    const shadow = this.sun.shadow;
    if (shadow.mapSize.x !== size) {
      shadow.mapSize.set(size, size);
      // 分辨率改变后需要重新创建阴影贴图
      shadow.map?.dispose();
      shadow.map = null;
    }
    const cam = shadow.camera;
    const r = this.shadow.halfExtent;
    cam.left = -r;
    cam.right = r;
    cam.top = r;
    cam.bottom = -r;
    cam.near = 0.5;
    cam.far = this.shadow.distance * 2;
    cam.updateProjectionMatrix();
  }

  get shadowQuality(): ShadowQuality {
    return this.quality;
  }

  /**
   * 应用光照预设。传入 scene 时一并设置背景色、雾与环境贴图强度。
   */
  applyPreset(name: LightingPresetName | LightingPreset, scene?: THREE.Scene): void {
    const p = typeof name === 'string' ? lightingPresets[name] : name;
    this.presetName = typeof name === 'string' ? name : null;
    this.sun.color.set(p.sun.color);
    this.sun.intensity = p.sun.intensity;
    this.sunShadowEnabled = p.sun.castShadow;
    this.sun.castShadow = p.sun.castShadow && this.quality !== 'off';
    this.setSunAngles(p.sun.azimuth, p.sun.elevation);
    this.hemisphere.color.set(p.hemisphere.sky);
    this.hemisphere.groundColor.set(p.hemisphere.ground);
    this.hemisphere.intensity = p.hemisphere.intensity;
    this.ambient.color.set(p.ambient.color);
    this.ambient.intensity = p.ambient.intensity;
    if (scene) {
      if (scene.background instanceof THREE.Color) scene.background.set(p.background);
      else scene.background = new THREE.Color(p.background);
      scene.fog = p.fog ? new THREE.Fog(p.fog.color, p.fog.near, p.fog.far) : null;
      scene.environmentIntensity = p.environmentIntensity;
    }
  }

  /**
   * 启用环境贴图（基于 RoomEnvironment 生成的 PMREM），为金属/光滑材质提供反射与间接光。
   * 需要 WebGL 渲染器，重复调用会复用已生成的贴图。
   */
  enableEnvironment(renderer: THREE.WebGLRenderer, scene: THREE.Scene): void {
    if (!this.environmentMap) {
      const pmrem = new THREE.PMREMGenerator(renderer);
      const room = new RoomEnvironment();
      this.environmentMap = pmrem.fromScene(room, 0.04).texture;
      room.dispose();
      pmrem.dispose();
    }
    scene.environment = this.environmentMap;
  }

  disableEnvironment(scene: THREE.Scene): void {
    if (scene.environment === this.environmentMap) scene.environment = null;
  }

  /** 添加点光源 */
  addPointLight(options: PointLightOptions = {}): THREE.PointLight {
    const light = new THREE.PointLight(
      options.color ?? 0xffffff,
      options.intensity ?? 10,
      options.distance ?? 0,
      options.decay ?? 2,
    );
    if (options.position) light.position.copy(options.position);
    this.configureShadow(light, options);
    this.extras.add(light);
    this.group.add(light);
    return light;
  }

  /** 添加聚光灯 */
  addSpotLight(options: SpotLightOptions = {}): THREE.SpotLight {
    const light = new THREE.SpotLight(
      options.color ?? 0xffffff,
      options.intensity ?? 30,
      options.distance ?? 0,
      options.angle ?? Math.PI / 6,
      options.penumbra ?? 0.3,
      options.decay ?? 2,
    );
    if (options.position) light.position.copy(options.position);
    if (options.target) light.target.position.copy(options.target);
    this.configureShadow(light, options);
    this.extras.add(light);
    this.group.add(light, light.target);
    return light;
  }

  /** 移除一个点光源 / 聚光灯 */
  removeLight(light: THREE.PointLight | THREE.SpotLight): void {
    if (!this.extras.delete(light)) return;
    this.attachments.delete(light);
    this.group.remove(light);
    if (light instanceof THREE.SpotLight) this.group.remove(light.target);
    light.dispose();
  }

  /** 移除所有点光源与聚光灯（切换场景时使用） */
  clearLights(): void {
    for (const light of [...this.extras]) this.removeLight(light);
  }

  /** 当前的点光源与聚光灯 */
  get lights(): readonly (THREE.PointLight | THREE.SpotLight)[] {
    return [...this.extras];
  }

  /**
   * 把光源绑定到刚体：每次 update 时按刚体（插值后的）位姿更新光源位置，
   * 聚光灯的照射方向随刚体旋转。刚体被销毁后绑定自动解除。
   */
  attachToBody(
    light: THREE.PointLight | THREE.SpotLight,
    body: RigidBody,
    options: AttachOptions = {},
  ): void {
    const o = options.offset;
    const d = options.direction;
    const direction = d ? new Vec3(d.x, d.y, d.z) : new Vec3(0, -1, 0);
    direction.normalize();
    this.attachments.set(light, {
      body,
      offset: o ? new Vec3(o.x, o.y, o.z) : new Vec3(),
      direction,
    });
  }

  detach(light: THREE.Light): void {
    this.attachments.delete(light);
  }

  /**
   * 每帧渲染前调用。
   * @param focus 主光阴影的跟随中心（一般为相机注视点）
   * @param alpha 物理插值系数，与 PhysicsView.sync 使用相同的值
   */
  update(focus?: THREE.Vector3Like, alpha = 1): void {
    for (const [light, a] of this.attachments) {
      const body = a.body;
      if (!body.world) {
        this.attachments.delete(light);
        continue;
      }
      body.interpolate(alpha, tmpP, tmpQ);
      tmpQ.rotate(a.offset, tmpV).add(tmpP);
      light.position.set(tmpV.x, tmpV.y, tmpV.z);
      if (light instanceof THREE.SpotLight) {
        tmpQ.rotate(a.direction, tmpV);
        light.target.position.set(
          light.position.x + tmpV.x,
          light.position.y + tmpV.y,
          light.position.z + tmpV.z,
        );
      }
    }
    if (focus) this.shadowCenter.copy(focus);
    this.updateSunTransform();
  }

  /** 释放环境贴图与所有光源 */
  dispose(): void {
    this.clearLights();
    this.environmentMap?.dispose();
    this.environmentMap = null;
    this.sun.dispose();
    this.hemisphere.dispose();
    this.ambient.dispose();
  }

  private updateSunTransform(): void {
    const mapSize = this.sun.shadow.mapSize.x;
    snapShadowCenter(
      this.shadowCenter,
      this.sunDirection,
      this.shadow.halfExtent,
      mapSize,
      tmpFocus,
    );
    this.sun.target.position.copy(tmpFocus);
    this.sun.position.copy(tmpFocus).addScaledVector(this.sunDirection, this.shadow.distance);
  }

  private configureShadow(
    light: THREE.PointLight | THREE.SpotLight,
    options: PointLightOptions,
  ): void {
    light.castShadow = options.castShadow ?? false;
    if (light.castShadow) {
      const size = options.shadowMapSize ?? 512;
      light.shadow.mapSize.set(size, size);
      light.shadow.bias = -0.0005;
      light.shadow.normalBias = 0.02;
      light.shadow.radius = 2;
      light.shadow.camera.near = 0.1;
    }
  }
}
