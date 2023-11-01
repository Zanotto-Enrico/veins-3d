import re
# Leggi il file .net
with open('erlangen.net.xml', 'r') as f:
    data = f.read()
# Crea un nuovo file .net contenente solo le strade nel primo quadrante
with open('filtered.net.xml', 'w') as f:
    lines = data.split('\n')    
    for line in lines:
        if line.startswith('edge'):            
            match = re.search(r'(\d+\.\d+)\s+(\d+\.\d+)\s+(\d+\.\d+)\s+(\d+\.\d+)', line)
            if match:                
                x1, y1, x2, y2 = map(float, match.groups())
                if x1 < 6500 and x2 < 6500 and y1 < 6500 and y2 < 6500:
                    f.write(line + '\n')
        else:            
            f.write(line + '\n')
# Usa randomTrips.py con il nuovo file .net filtrato
# Esempio:# randomTrips.py -n filtered_net_file.net -o output_routes.xml --prefix vehicle -l 1000 -s 0
