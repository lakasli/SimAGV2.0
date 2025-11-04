/**
 * 路径规划与 VDA 5050 Order 生成（基于站点，不使用锚点）。
 * 移植并对齐至 Python 侧的站点化实现。
 */

/**
 * 解析从后端获取的 scene 地图文件内容（构建站点与路段）。
 * @param {object} sceneData - 地图文件 JSON 对象.
 * @returns {{stations: object[], paths: object[]}}
 */
function parseSceneTopology(sceneData) {
  const root = Array.isArray(sceneData) ? sceneData[0] : sceneData;
  if (!root) return { stations: [], paths: [] };

  const points = root.points || [];
  const routes = root.routes || [];

  // 使用“站点”统一指代所有图节点（不再区分锚点）。
  const stationsMap = new Map();
  for (const p of points) {
    const pid = p.id != null ? String(p.id) : null;
    if (!pid) continue;
    stationsMap.set(pid, {
      id: pid,
      name: String(p.name || pid),
      x: Number(p.x || 0),
      y: Number(p.y || 0),
      // 兼容地图点位可能包含的朝向字段（theta/orientation），缺省为 0
      theta: (typeof p.theta === 'number') ? Number(p.theta) : (typeof p.orientation === 'number' ? Number(p.orientation) : 0),
    });
  }

  const paths = [];
  for (const r of routes) {
    const fromId = r.from != null ? String(r.from) : null;
    const toId = r.to != null ? String(r.to) : null;
    if (!fromId || !toId || !stationsMap.has(fromId) || !stationsMap.has(toId)) {
      continue;
    }
    const p0 = stationsMap.get(fromId);
    const p3 = stationsMap.get(toId);
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
    // 路径规划的图节点统一使用“站点”（包含所有 points）
    stations: Array.from(stationsMap.values()),
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
 * A* 路径规划（基于站点）。
 * @param {string} startId - 起始站点 ID。
 * @param {string} endId - 终点站点 ID。
 * @param {object[]} stations - 站点列表。
 * @param {object[]} paths - 路段列表。
 * @returns {string[] | null} - 节点 ID 路径, 或 null (无路径)。
 */
function aStar(startId, endId, stations, paths) {
  const stationsMap = new Map(stations.map(s => [s.id, s]));
  const adj = new Map();
  for (const p of paths) {
    if (!adj.has(p.from)) adj.set(p.from, []);
    adj.get(p.from).push({ node: p.to, weight: p.length });
  }

  const heuristic = (a, b) => {
    const pa = stationsMap.get(a);
    const pb = stationsMap.get(b);
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
 * 寻找离指定位置最近的站点。
 * @param {{x: number, y: number}} pos
 * @param {object[]} stations
 * @returns {string | null}
 */
function findNearestStation(pos, stations) {
  let bestId = null;
  let bestDist = Infinity;
  for (const s of stations) {
    const dist = Math.hypot(s.x - pos.x, s.y - pos.y);
    if (dist < bestDist) {
      bestDist = dist;
      bestId = s.id;
    }
  }
  return bestId;
}

/**
 * 生成 VDA 5050 订单（节点为站点名称）。
 * @param {string[]} path - A* 规划出的节点 ID 列表（站点 ID）。
 * @param {{serial_number: string, manufacturer: string, version?: string}} agvInfo - 车辆信息.
 * @param {{stations: object[], paths: object[]}} topo - 地图拓扑（需含 stations.name 与 paths.desc）。
 * @returns {object} - VDA 5050 Order.
 */
function generateVdaOrder(path, agvInfo, topo, mapId, npOptions = {}) {
  const timestamp = new Date().toISOString();
  const orderId = 'order-' + timestamp.replace(/\D/g, '');

  const stationById = new Map((topo?.stations || []).map(s => [String(s.id), s]));
  const nodeNames = path.map(id => {
    const s = stationById.get(String(id));
    return s && s.name ? String(s.name) : String(id);
  });

  // camelCase 节点
  const nodes = nodeNames.map((name, i) => {
    const origId = String(path[i]);
    const s = stationById.get(origId) || { x: 0, y: 0, theta: 0 };
    const allowedDeviationXY = (typeof npOptions.allowedDeviationXY === 'number') ? Number(npOptions.allowedDeviationXY) : 0.5;
    const allowedDeviationTheta = (typeof npOptions.allowedDeviationTheta === 'number') ? Number(npOptions.allowedDeviationTheta) : 0.5;
    const nodePosition = {
      x: Number(s.x || 0),
      y: Number(s.y || 0),
      theta: (typeof s.theta === 'number') ? Number(s.theta) : 0,
      allowedDeviationXY,
      allowedDeviationTheta,
      mapId: String(mapId || ''),
      // mapDescription 暂无来源，保持为空以兼容
    };
    return {
      nodeId: name,
      sequenceId: i + 1,
      released: true,
      actions: [],
      nodePosition,
    };
  });

  // camelCase 边
  const edges = [];
  for (let i = 0; i < path.length - 1; i++) {
    // 查找路段描述（desc）
    const fromId = String(path[i]);
    const toId = String(path[i + 1]);
    const match = (topo?.paths || []).find(p => String(p.from) === fromId && String(p.to) === toId);
    const edgeDesc = match && typeof match.desc === 'string' ? match.desc : `${nodeNames[i]}-${nodeNames[i + 1]}`;
    edges.push({
      edgeId: edgeDesc,
      sequenceId: i + 1,
      released: true,
      startNodeId: nodeNames[i],
      endNodeId: nodeNames[i + 1],
      actions: [],
    });
  }

  // camelCase 订单头与字段
  return {
    headerId: Date.now() % 100000,
    timestamp,
    version: agvInfo.version || '2.0.0',
    manufacturer: agvInfo.manufacturer,
    serialNumber: agvInfo.serial_number,
    orderId: orderId,
    orderUpdateId: 0,
    nodes,
    edges,
  };
}