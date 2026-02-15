from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import requests
import math

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*')

# DURUM DEĞİŞKENLERİ
waypoint_queue   = []
waypoint_history = []  

current_target = {'lat': None, 'lng': None, 'updated': False}

vehicle_state = {
    'lat': None, 'lng': None,
    'heading': 0.0, 'speed': 0.0,
    'dist_to_target': 0.0,
    'waypoints_remaining': 0,
    'obstacle_mode': 'NORMAL',
    'status': 'BEKLIYOR'
}

# Webots'un okuduğu komut kanalı
vehicle_command = {'action': 'RUN', 'updated': False}
paused = False


# YARDIMCI 
def haversine(lat1, lng1, lat2, lng2):
    R = 6371000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lng2 - lng1)
    a  = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def get_osrm_route(start_lat, start_lng, end_lat, end_lng):
    url = (
        f"http://router.project-osrm.org/route/v1/driving/"
        f"{start_lng},{start_lat};{end_lng},{end_lat}"
        f"?overview=full&geometries=geojson&steps=true"
    )
    try:
        r    = requests.get(url, timeout=5)
        data = r.json()
        if data.get('code') != 'Ok':
            return None
        coords = data['routes'][0]['geometry']['coordinates']
        return {
            'waypoints':  [{'lat': c[1], 'lng': c[0]} for c in coords],
            'distance_m': data['routes'][0]['distance'],
            'duration_s': data['routes'][0]['duration']
        }
    except Exception as e:
        print(f"OSRM hatası: {e}")
        return None


def derive_status(state):
    mode = state.get('obstacle_mode', 'NORMAL')
    if mode in ('EMERGENCY', 'AVOIDING'): return 'ENGEL'
    if paused:                            return 'DURDU'
    if (state.get('speed') or 0) > 0.5:  return 'HAREKET'
    if state.get('lat'):                  return 'BEKLIYOR'
    return 'BEKLIYOR'


# FLASK ROUTE'LARI
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/get_target', methods=['GET'])
def get_target():
    global current_target
    resp = jsonify(current_target)
    current_target['updated'] = False
    return resp


@app.route('/get_command', methods=['GET'])
def get_command():
    """Webots araç komutunu buradan okur: RUN / PAUSE / CLEAR"""
    global vehicle_command
    resp = jsonify(vehicle_command)
    vehicle_command['updated'] = False
    return resp


@app.route('/vehicle_update', methods=['POST'])
def vehicle_update():
    global vehicle_state
    data = request.get_json()
    vehicle_state.update(data)
    vehicle_state['waypoints_remaining'] = len(waypoint_queue)
    vehicle_state['status'] = derive_status(vehicle_state)
    socketio.emit('vehicle_update', vehicle_state)
    return jsonify({'ok': True})


@app.route('/waypoint_reached', methods=['POST'])
def waypoint_reached():
    global waypoint_queue, current_target, waypoint_history
    reached = request.get_json().get('waypoint', {})

    if reached.get('lat') and reached.get('lng'):
        waypoint_history.append(reached)

    socketio.emit('waypoint_reached', reached)

    if waypoint_queue:
        nxt = waypoint_queue.pop(0)
        current_target.update({'lat': nxt['lat'], 'lng': nxt['lng'], 'updated': True})
        socketio.emit('next_waypoint', {'waypoint': nxt, 'remaining': len(waypoint_queue)})
        print(f"Sonraki waypoint: {nxt} | Kalan: {len(waypoint_queue)}")
    else:
        current_target.update({'lat': None, 'lng': None, 'updated': False})
        vehicle_state['status'] = 'TAMAMLANDI'
        socketio.emit('route_complete', {})
        socketio.emit('sistem_mesaji', {'msg': '🏁 Rota tamamlandı!', 'type': 'success'})
        print("Rota tamamlandı!")

    return jsonify({'ok': True, 'remaining': len(waypoint_queue)})


# SOCKET.IO — BAĞLANTI 
@socketio.on('connect')
def on_connect():
    emit('vehicle_update', vehicle_state)
    emit('queue_update', {
        'count':   len(waypoint_queue),
        'paused':  paused,
        'command': vehicle_command['action']
    })


# SOCKET.IO — ROTA
@socketio.on('plan_route')
def handle_plan_route(data):
    global waypoint_queue, current_target, waypoint_history

    emit('sistem_mesaji', {'msg': 'OSRM rotası hesaplanıyor...', 'type': 'info'})
    result = get_osrm_route(
        data.get('start_lat'), data.get('start_lng'),
        data.get('end_lat'),   data.get('end_lng')
    )
    if result is None:
        emit('sistem_mesaji', {'msg': 'OSRM bağlantı hatası!', 'type': 'error'})
        return

    all_wps  = result['waypoints']
    filtered = [all_wps[0]]
    for wp in all_wps[1:]:
        if haversine(filtered[-1]['lat'], filtered[-1]['lng'],
                     wp['lat'], wp['lng']) >= 10:
            filtered.append(wp)

    waypoint_history = []
    waypoint_queue   = filtered[1:]

    if waypoint_queue:
        first = waypoint_queue.pop(0)
        current_target.update({'lat': first['lat'], 'lng': first['lng'], 'updated': True})

    dist_km = result['distance_m'] / 1000
    dur_min = result['duration_s'] / 60
    msg     = f"Rota hazır — {len(filtered)} waypoint | {dist_km:.1f} km | ~{dur_min:.0f} dk"
    print(msg)
    emit('sistem_mesaji', {'msg': msg, 'type': 'success'})
    emit('route_planned', {
        'waypoints':  filtered,
        'distance_m': result['distance_m'],
        'duration_s': result['duration_s']
    })


@socketio.on('hedef_belirle')
def handle_single_target(data):
    global current_target, waypoint_queue, waypoint_history
    waypoint_queue = []
    waypoint_history = []
    current_target.update({'lat': data['lat'], 'lng': data['lng'], 'updated': True})
    emit('sistem_mesaji', {'msg': 'Hedef araca iletildi.', 'type': 'success'})


# SOCKET.IO — ARAÇ KOMUTLARI 
@socketio.on('pause_vehicle')
def handle_pause():
    global paused, vehicle_command
    paused = True
    vehicle_command.update({'action': 'PAUSE', 'updated': True})
    vehicle_state['status'] = 'DURDU'
    socketio.emit('vehicle_status_change', {'status': 'DURDU', 'paused': True})
    socketio.emit('sistem_mesaji', {'msg': '⏸ Araç duraklatıldı.', 'type': 'warning'})
    print("DURAKLAT komutu gönderildi.")


@socketio.on('resume_vehicle')
def handle_resume():
    global paused, vehicle_command
    paused = False
    vehicle_command.update({'action': 'RUN', 'updated': True})
    vehicle_state['status'] = 'HAREKET'
    socketio.emit('vehicle_status_change', {'status': 'HAREKET', 'paused': False})
    socketio.emit('sistem_mesaji', {'msg': '▶ Araç devam ediyor.', 'type': 'success'})
    print("DEVAM komutu gönderildi.")


@socketio.on('clear_route')
def handle_clear_route():
    global waypoint_queue, current_target, waypoint_history, paused, vehicle_command
    waypoint_queue   = []
    waypoint_history = []
    paused           = False
    current_target.update({'lat': None, 'lng': None, 'updated': False})
    vehicle_command.update({'action': 'CLEAR', 'updated': True})
    vehicle_state['status'] = 'BEKLIYOR'
    socketio.emit('route_cleared', {})
    socketio.emit('vehicle_status_change', {'status': 'BEKLIYOR', 'paused': False})
    socketio.emit('sistem_mesaji', {'msg': '🗑 Rota temizlendi.', 'type': 'warning'})
    print("ROTA TEMİZLE komutu gönderildi.")


@socketio.on('stop_vehicle')
def handle_stop():
    handle_clear_route()


if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)