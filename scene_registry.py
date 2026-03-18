# 处理场景资源注册 方便统一管理
import math
import re

class SceneRegistry:
    def __init__(self, client):
        self.client = client
    # 获取所有带Tag的actor
    def list_all_tagged_objects(self):
        return self.client.simListSceneObjectsTags(".*")
    # 通过查找Tag确定唯一的actor
    def find_unique_by_tag(self, tag: str):
        regex = f"^{re.escape(tag)}$"
        results = self.client.simListSceneObjectsTags(regex)

        if len(results) == 0:
            raise RuntimeError(f"未找到 tag={tag} 的对象")

        if len(results) > 1:
            raise RuntimeError(f"tag={tag} 对应多个对象: {results}")

        actor_name, actor_tag = results[0]
        return actor_name, actor_tag
    # 注册该资源
    def register_tags(self, tags):
        mapping = {}
        for tag in tags:
            actor_name, actor_tag = self.find_unique_by_tag(tag)
            mapping[tag] = actor_name
        return mapping
    # 得到查找物的详细信息 比如UE中的全局坐标
    def get_actor_world_position(self, actor_name: str):
        pose = self.client.simGetObjectPose(actor_name)
        p = pose.position
        x = p.x_val
        y = p.y_val
        z = p.z_val
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            raise RuntimeError(
                f"对象 {actor_name} 世界坐标非法: ({x}, {y}, {z})"
            )
        return (x, y, z)