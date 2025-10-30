/**
 * 路径规划与 VDA 5050 Order 生成.
 * 移植自 `SimVehicleSys/sim_vehicle/navigation.py`.
 */

/**
 * 解析从后端获取的 scene 地图文件内容.
 * @param {object} sceneData - 地图文件 JSON 对象.
 * @returns {{stations: object[], anchors: object[], paths: object[]}}
 */
function parseSceneTopology(sceneData) {
  const root = Array.isArray(sceneData) ? sceneData[0] : sceneData;
  if (!root) return { stations: [], anchors: [], paths: [] };

  const points = root.points || [];
  const routes = root.routes || [];

  const anchorsMap = new Map();
  for (const p of points) {
    const pid = p.id != null ? String(p.id) : null;
    if (!pid) continue;
    anchorsMap.set(pid, { id: pid, name: String(p.name || pid), x: Number(p.x || 0), y: Number(p.y || 0) });
  }

  const paths = [];
  for (const r of routes) {
    const fromId = r.from != null ? String(r.from) : null;
    const toId = r.to != null ? String(r.to) : null;
    if (!fromId || !toId || !anchorsMap.has(fromId) || !anchorsMap.has(toId)) {
      continue;
    }
    const p0 = anchorsMap.get(fromId);
    const p3 = anchorsMap.get(toId);
    const length = Math.hypot(p3.x - p0.x, p3.y - p0.y); // 简化为直线距离
    paths.push({
      id: r.id || `${fromId}->${toId}`,
      from: fromId,
      to: toId,
      desc: typeof r.desc === 'string' ? r.desc : `${p0.name}-${p3.name}`,
      length: length,
    });
  }

  return {
    stations: root.points.filter(p => /^(AP|CP|PP|LM|WP)/.test(p.name || '')).map(p => ({
      id: p.id != null ? String(p.id) : p.name,
      name: p.name,
      x: Number(p.x || 0),
      y: Number(p.y || 0),
    })),
    anchors: Array.from(anchorsMap.values()),
    paths: paths,
  };
}

/**
 * 简单的优先队列实现.
 */
class PriorityQueue {
  constructor() {
    this.elements = [];
  }
  enqueue(priority, element) {
    this.elements.push({ priority, element });
    this.elements.sort((a, b) => a.priority - b.priority);
  }
  dequeue() {
    return this.elements.shift()?.element;
  }
  isEmpty() {
    return this.elements.length === 0;
  }
}

/**
 * A* 路径规划.
 * @param {string} startId - 起始锚点ID.
 * @param {string} endId - 终点锚点ID.
 * @param {object[]} anchors - 锚点列表.
 * @param {object[]} paths - 路径列表.
 * @returns {string[] | null} - 节点ID路径, 或 null (无路径).
 */
function aStar(startId, endId, anchors, paths) {
  const anchorsMap = new Map(anchors.map(a => [a.id, a]));
  const adj = new Map();
  for (const p of paths) {
    if (!adj.has(p.from)) adj.set(p.from, []);
    adj.get(p.from).push({ node: p.to, weight: p.length });
  }

  const heuristic = (a, b) => {
    const pa = anchorsMap.get(a);
    const pb = anchorsMap.get(b);
    return pa && pb ? Math.hypot(pa.x - pb.x, pa.y - pb.y) : 0;
  };

  const openSet = new PriorityQueue();
  openSet.enqueue(0, startId);

  const cameFrom = new Map();
  const gScore = new Map();
  gScore.set(startId, 0);

  while (!openSet.isEmpty()) {
    const current = openSet.dequeue();

    if (current === endId) {
      const path = [];
      let temp = current;
      while (temp) {
        path.unshift(temp);
        temp = cameFrom.get(temp);
      }
      return path;
    }

    const neighbors = adj.get(current) || [];
    for (const { node: neighbor, weight } of neighbors) {
      const tentativeGScore = (gScore.get(current) ?? Infinity) + weight;
      if (tentativeGScore < (gScore.get(neighbor) ?? Infinity)) {
        cameFrom.set(neighbor, current);
        gScore.set(neighbor, tentativeGScore);
        const fScore = tentativeGScore + heuristic(neighbor, endId);
        openSet.enqueue(fScore, neighbor);
      }
    }
  }

  return null; // No path found
}

/**
 * 寻找离指定位置最近的锚点.
 * @param {{x: number, y: number}} pos
 * @param {object[]} anchors
 * @returns {string | null}
 */
function findNearestAnchor(pos, anchors) {
  let bestId = null;
  let bestDist = Infinity;
  for (const a of anchors) {
    const dist = Math.hypot(a.x - pos.x, a.y - pos.y);
    if (dist < bestDist) {
      bestDist = dist;
      bestId = a.id;
    }
  }
  return bestId;
}

/**
 * 生成 VDA 5050 订单.
 * @param {string[]} path - A* 规划出的节点ID列表（锚点 ID）。
 * @param {{serial_number: string, manufacturer: string, version?: string}} agvInfo - 车辆信息.
 * @param {{anchors: object[], paths: object[]}} topo - 地图拓扑（需含 anchors.name 与 paths.desc）。
 * @returns {object} - VDA 5050 Order.
 */
function generateVdaOrder(path, agvInfo, topo) {
  const timestamp = new Date().toISOString();
  const orderId = 'order-' + timestamp.replace(/\D/g, '');

  const anchorById = new Map((topo?.anchors || []).map(a => [String(a.id), a]));
  const nodeNames = path.map(id => {
    const a = anchorById.get(String(id));
    return a && a.name ? String(a.name) : String(id);
  });

  const nodes = nodeNames.map((name, i) => ({
    node_id: name,
    sequence_id: i + 1,
    released: true,
  }));

  const edges = [];
  for (let i = 0; i < path.length - 1; i++) {
    // 查找路段描述（desc）
    const fromId = String(path[i]);
    const toId = String(path[i + 1]);
    const match = (topo?.paths || []).find(p => String(p.from) === fromId && String(p.to) === toId);
    const edgeDesc = match && typeof match.desc === 'string' ? match.desc : `${nodeNames[i]}-${nodeNames[i + 1]}`;
    edges.push({
      edge_id: edgeDesc,
      sequence_id: i + 1,
      released: true,
      start_node_id: nodeNames[i],
      end_node_id: nodeNames[i + 1],
    });
  }

  return {
    header_id: Date.now() % 100000,
    timestamp,
    version: agvInfo.version || '2.0.0',
    manufacturer: agvInfo.manufacturer,
    serial_number: agvInfo.serial_number,
    order_id: orderId,
    order_update_id: 0,
    // 按 VDA 2.0 Order 要求必须提供该键，允许为 null
    zone_set_id: null,
    nodes,
    edges,
  };
}