
#    <link rel="stylesheet" href="{{ url_for('static', filename='styles.css') }}">

import pandas as pd
from scipy.spatial.distance import cdist
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import folium

# CSV dosyasını yükleme
file_path = '3ankara.csv'  # Dosya yolunu doğru şekilde güncelleyin
ankara_data = pd.read_csv(file_path)

# Müşteri koordinatlarını alma
coordinates = ankara_data[['enlem', 'boylam']].values

# Mesafe matrisini hesaplama (km cinsinden yaklaşık mesafeler)
distance_matrix = cdist(coordinates, coordinates, metric='euclidean') * 111  # 1 derece ~111 km

# Veri hazırlığı: TSP için mesafe matrisi
def create_data_model():
    """Veri modelini oluşturur."""
    data = {
        'distance_matrix': distance_matrix.astype(int).tolist(),  # Mesafe matrisini tam sayıya çevir
        'num_locations': len(distance_matrix),  # Konum sayısı
        'depot': 0,  # Başlangıç noktası (ilk müşteri)
    }
    return data

# Veri modeli
data = create_data_model()

# Rota yöneticisi oluşturma
manager = pywrapcp.RoutingIndexManager(data['num_locations'], 1, data['depot'])

# Routing modeli oluşturma
routing = pywrapcp.RoutingModel(manager)

# Mesafe hesaplayıcı
def distance_callback(from_index, to_index):
    """İki düğüm arasındaki mesafeyi döndürür."""
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['distance_matrix'][from_node][to_node]

# Mesafe hesaplayıcıyı modele bağlama
transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Çözüm stratejisini belirleme
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

# Modeli çözme
solution = routing.SolveWithParameters(search_parameters)

# Rotayı alma
def get_solution(manager, routing, solution):
    """Çözümdeki rotayı döndürür."""
    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        route.append(node)
        index = solution.Value(routing.NextVar(index))
    route.append(manager.IndexToNode(index))  # Döngüyü kapatma
    return route

# Çözüm varsa rotayı gösterme
if solution:
    route = get_solution(manager, routing, solution)
    optimized_route = [ankara_data['müşteri_ismi'].iloc[i] for i in route[:-1]]  # Dönüş noktası hariç
else:
    optimized_route = []

# Rotayı yazdırma
print("Optimizasyon Sonucu Rota:")
print(" -> ".join(optimized_route))

# Haritada rotayı gösterme
ankara_map = folium.Map(location=[39.92077, 32.85411], zoom_start=10)  # Ankara merkez
route_coordinates = [coordinates[i] for i in route]

# Rotayı çizen çizgi
folium.PolyLine(route_coordinates, color="blue", weight=2.5, opacity=1).add_to(ankara_map)

# Müşteri konumlarını işaretleme
for i, (lat, lon) in enumerate(route_coordinates):
    folium.Marker(
        location=[lat, lon],
        popup=f"{ankara_data['müşteri_ismi'].iloc[route[i]]}",
        tooltip=f"Sıra {i+1}"
    ).add_to(ankara_map)

# Haritayı kaydetme
ankara_map.save("optimized_ankara_route.html")
print("Rota haritası 'optimized_ankara_route.html' olarak kaydedildi.")
