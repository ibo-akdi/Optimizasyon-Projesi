from flask import Flask, render_template, request, flash, url_for   
import pandas as pd
import numpy as np  
from math import radians, sin, cos, sqrt, atan2
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
import folium
import os

app = Flask(__name__)
app.secret_key = 'your_secret_key'

UPLOAD_FOLDER = 'uploads'
if not os.path.exists(UPLOAD_FOLDER):
    os.makedirs(UPLOAD_FOLDER)

app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# Distance calculation (Haversine formula)
def haversine(coord1, coord2):
    R = 6371  # Radius of Earth in kilometers
    lat1, lon1 = radians(coord1[0]), radians(coord1[1])
    lat2, lon2 = radians(coord2[0]), radians(coord2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c * 1000  # Convert to meters

# Create distance matrix
def create_distance_matrix(latitudes, longitudes):
    n = len(latitudes)
    distance_matrix = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if i != j:
                coord1 = (latitudes[i], longitudes[i])
                coord2 = (latitudes[j], longitudes[j])
                distance_matrix[i][j] = haversine(coord1, coord2)
    return distance_matrix

# Group customers by capacity
def group_customers_by_capacity(demands, vehicle_capacity):
    groups = []
    current_group = []
    current_load = 0

    for i, demand in enumerate(demands):
        if current_load + demand <= vehicle_capacity:
            current_group.append(i)
            current_load += demand
        else:
            groups.append(current_group)
            current_group = [i]
            current_load = demand

    if current_group:
        groups.append(current_group)

    return groups

# Solve TSP with start and end
def solve_tsp_with_start_end(group, distance_matrix):
    group = [0] + group + [0]

    manager = pywrapcp.RoutingIndexManager(len(group), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return int(distance_matrix[group[manager.IndexToNode(from_index)]][group[manager.IndexToNode(to_index)]])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        route = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route.append(group[node])
            index = solution.Value(routing.NextVar(index))
        route.append(group[0])  # Return to start
        return route
    return None

# Home page
@app.route('/')
def home():
    return render_template('index.html')

# Optimization page
@app.route('/optimize', methods=['POST'])
def optimize():
    try:
        csv_file = request.files.get('file')
        vehicle_capacity = int(request.form.get('vehicle_capacity', 100))
        max_cost = float(request.form.get('max_cost', 10000))
        cost_per_km = float(request.form.get('cost_per_km', 5)) / 1000  # Convert TL per meter
        new_customer_lat = request.form.get('new_customer_lat', None)
        new_customer_lon = request.form.get('new_customer_lon', None)
        new_customer_demand = request.form.get('new_customer_demand', None)
        if not csv_file:
            flash("Lütfen geçerli bir CSV dosyası yükleyin!")
            return render_template('index.html')
        # Yüklenen dosyayı kaydet
        csv_path = os.path.join(app.config['UPLOAD_FOLDER'], csv_file.filename)
        csv_file.save(csv_path)
        # CSV verilerini oku
        data = pd.read_csv(csv_path)

        # Yeni müşteri ekle
        if new_customer_lat and new_customer_lon and new_customer_demand:
            new_customer = {
                'enlem': float(new_customer_lat),
                'boylam': float(new_customer_lon),
                'malzeme_talebi': int(new_customer_demand)
            }
            data = pd.concat([data, pd.DataFrame([new_customer])], ignore_index=True)
            data.to_csv(csv_path, index=False)
            flash("Yeni müşteri CSV dosyasına eklendi.")

        # Verilerden listeler oluştur
        latitudes = data['enlem'].tolist()
        longitudes = data['boylam'].tolist()
        demands = data['malzeme_talebi'].tolist()

        # Depo noktası ekle
        depot_lat, depot_lon = 39.9208, 32.8541
        latitudes.insert(0, depot_lat)
        longitudes.insert(0, depot_lon)
        demands.insert(0, 0)
        # Mesafe matrisi oluştur
        distance_matrix = create_distance_matrix(latitudes, longitudes)
        # Kapasiteye göre müşterileri gruplandır
        groups = group_customers_by_capacity(demands[1:], vehicle_capacity)
        route_maps = []
        for idx, group in enumerate(groups):
            optimal_route = solve_tsp_with_start_end(group, distance_matrix)
            if optimal_route:
                total_distance = sum(distance_matrix[optimal_route[i]][optimal_route[i + 1]] for i in range(len(optimal_route) - 1))
                total_cost = total_distance * cost_per_km

                if total_cost > max_cost:
                    flash(f"Grup {idx + 1} maliyeti ({total_cost:.2f} TL) maksimum maliyeti ({max_cost} TL) aşıyor. Alt gruplar oluşturuluyor.")

                    # Alt gruplara böl
                    sub_groups = group_customers_by_capacity([demands[i] for i in group], vehicle_capacity // 2)

                    for sub_idx, sub_group in enumerate(sub_groups):
                        sub_group = [group[i] for i in sub_group]
                        sub_route = solve_tsp_with_start_end(sub_group, distance_matrix)

                        if sub_route:
                            sub_distance = sum(distance_matrix[sub_route[i]][sub_route[i + 1]] for i in range(len(sub_route) - 1))
                            sub_cost = sub_distance * cost_per_km
                            flash(f"Alt Grup {idx + 1}-{sub_idx + 1} için toplam maliyet: {sub_cost:.2f} TL.")

                            group_map = folium.Map(location=[depot_lat, depot_lon], zoom_start=12)

                            # Başlangıç noktasından ilk müşteriye giden çizgi (örneğin, mavi)
                            if len(sub_route) > 1:
                                start, end = 0, sub_route[0]
                                folium.PolyLine(
                                    [[latitudes[start], longitudes[start]], [latitudes[end], longitudes[end]]],
                                    color="blue", weight=2.5, opacity=1
                                ).add_to(group_map)

                            for i in range(len(sub_route) - 1):
                                start, end = sub_route[i], sub_route[i + 1]

                                # Son müşteriden başlangıç noktasına gelen çizgi (örneğin, yeşil)
                                if i == len(sub_route) - 2:
                                    line_color = "green"
                                # Diğer çizgiler (örneğin, kırmızı)
                                else:
                                    line_color = "red"

                                # Polyline oluştur
                                folium.PolyLine(
                                    [[latitudes[start], longitudes[start]], [latitudes[end], longitudes[end]]],
                                    color=line_color, weight=2.5, opacity=1
                                ).add_to(group_map)

                                # Her müşteri için bir marker ekle
                                folium.Marker(
                                    [latitudes[start], longitudes[start]],
                                    popup=f"Customer: {start}, Demand: {demands[start]}",
                                    icon=folium.Icon(color="blue"),
                                ).add_to(group_map)

                            # Son müşteriden başlangıç noktasına gelen çizgi (örneğin, yeşil)
                            if len(sub_route) > 1:
                                start, end = sub_route[-1], 0
                                folium.PolyLine(
                                    [[latitudes[start], longitudes[start]], [latitudes[end], longitudes[end]]],
                                    color="green", weight=2.5, opacity=1
                                ).add_to(group_map)

                            map_file_path = f"static/route_map_group_{idx + 1}_sub_{sub_idx + 1}.html"
                            group_map.save(map_file_path)
                            route_maps.append(url_for('static', filename=f'route_map_group_{idx + 1}_sub_{sub_idx + 1}.html'))
                else:
                    flash(f"Grup {idx + 1} için toplam maliyet: {total_cost:.2f} TL.")

                    group_map = folium.Map(location=[depot_lat, depot_lon], zoom_start=12)
                    for i in range(len(optimal_route) - 1):
                        start, end = optimal_route[i], optimal_route[i + 1]
                        folium.Marker(
                            [latitudes[start], longitudes[start]],
                            popup=f"Customer: {start}, Demand: {demands[start]}",
                            icon=folium.Icon(color="blue"),
                        ).add_to(group_map)
                        folium.PolyLine(
                            [[latitudes[start], longitudes[start]], [latitudes[end], longitudes[end]]],
                            color="red", weight=2.5, opacity=1
                        ).add_to(group_map)

                    map_file_path = f"static/route_map_group_{idx + 1}.html"
                    group_map.save(map_file_path)
                    route_maps.append(url_for('static', filename=f'route_map_group_{idx + 1}.html'))

        return render_template('results.html', route_maps=route_maps)

    except Exception as e:
        flash(f"Hata oluştu: {str(e)}")
        return render_template('index.html')

if __name__ == '__main__':
    app.run(debug=True)
