import xml.etree.ElementTree as ET
import math

# this script processes a the network file to identify road junctions. 
# It groups junctions based on proximity defined by a grid size (tolerance). 
# For groups with more than two junctions, it calculates RSU positions relative to the grid. 
# The positions are printed and written to rsus.txt. 
# This helps configure RSUs for vehicular simulations.


def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def parse_net_xml(file_path, tolerance=1000.0):
    tree = ET.parse(file_path)
    root = tree.getroot()

    crossings = root.findall(".//junction")
    crossings_count = {}

    minx = 10000000
    miny = 10000000
    for crossing in crossings:
        x_pos = float(crossing.attrib['x'])
        y_pos = float(crossing.attrib['y'])

        if (int(x_pos / tolerance)*tolerance, int(y_pos / tolerance)*tolerance) not in crossings_count.keys():
            crossings_count[(int(x_pos / tolerance)*tolerance, int(y_pos / tolerance)*tolerance)]= [(x_pos,y_pos)]     
        else:
            crossings_count[(int(x_pos / tolerance)*tolerance, int(y_pos / tolerance)*tolerance)].append((x_pos,y_pos))    
        
        if minx > x_pos:
            minx = x_pos
        if miny > y_pos:
            miny = y_pos

    n = 0
    file = open("rsus.txt","w")
    for key, value  in crossings_count.items():
        if len(value) > 2:
            print(f"*.rsu[{n}].mobility.x = {int(value[0][0] - minx) - 26}")
            print(f"*.rsu[{n}].mobility.y = {0 - int(value[0][1] - miny) + 1749}")
            file.write(f"*.rsu[{n}].mobility.x = {int(value[0][0] - minx) + 26}\n")
            file.write(f"*.rsu[{n}].mobility.y = {0 - int(value[0][1] - miny) + 1749}\n")
            
            n += 1
    print("crossing found: ", n)

if __name__ == '__main__':
    file_path = "test.net.xml"  
    size = input("grid size: ")
    parse_net_xml(file_path,  int(size))
    
    

