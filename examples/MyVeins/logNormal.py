import re
import numpy as np
from scipy.stats import lognorm

# Definizione dei parametri per la distribuzione log-normale
sigma = 0.9
mu = 3.04
np.random.seed(0)  # Imposta il seed per la riproducibilità

# Funzione per generare un nuovo valore per il parametro layer
def generate_new_layer_value():
    return round(generate_lognormal_values(sigma, mu, size=1)[0],3)

# Funzione per generare valori dalla distribuzione log-normale
def generate_lognormal_values(sigma, mu, size):
    return lognorm(s=sigma, scale=np.exp(mu)).rvs(size=size)

# Leggi il file e sostituisci i valori di layer
with open('test.poly.flat.xml', 'r') as f:
    content = f.read()

# Trova tutti i valori del parametro layer
layers = re.findall(r'layer="([-+]?\d*\.\d+|[-+]?\d+)"', content)

# Sostituisci i valori di layer con i nuovi valori generati
new_content = re.sub(r'layer="([-+]?\d*\.\d+|[-+]?\d+)"', lambda x: f'layer="{generate_new_layer_value()}"', content)

# Scrivi il nuovo contenuto nel file test2.poly.xml
with open('test.poly.3d.xml', 'w') as f:
    f.write(new_content)

print("File test.poly.3d.xml generato con successo.")
