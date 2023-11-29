import xml.etree.ElementTree as ET

# Let's assume the test.poly.xml file is in the same directory as this script.
tree = ET.parse('test.poly.xml')
root = tree.getroot()

# Extract building data
building_data = []
for poly in root.findall('poly'):
    building_id = poly.get('id')
    building_type = poly.get('type')
    building_layer = float(poly.get('layer'))
    building_shape = poly.get('shape')
    building_color = poly.get('color')
    building_coordinates = [tuple(map(float, point.split(','))) for point in building_shape.split()]

    if(building_color == '255,230,230'):
        building_data.append({
            'id': building_id,
            'type': building_type,
            'layer': building_layer,
            'shape': building_coordinates
        })

# Sort buildings by height
building_data_sorted = sorted(building_data, key=lambda x: x['layer'], reverse=True)

# Generate coordinates for the RSUs
rsu_coordinates = []
for i in range(51):
    building = building_data_sorted[i]
    avg_x = sum((point[0] + 26) for point in building['shape']) / len(building['shape'])
    avg_y = sum((1749 - point[1]) for point in building['shape']) / len(building['shape'])
    rsu_coordinates.append({
        'x': avg_x,
        'y': avg_y,
        'z': building['layer'] + 5
    })

# Print the generated coordinates
for i, rsu_coord in enumerate(rsu_coordinates):
    print(f'*.rsu[{i}].mobility.x = {rsu_coord["x"]}')
    print(f'*.rsu[{i}].mobility.y = {rsu_coord["y"]}')
    print(f'*.rsu[{i}].mobility.z = {rsu_coord["z"]}')
