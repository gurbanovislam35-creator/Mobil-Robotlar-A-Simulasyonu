import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math

# Hoca ödevde iki tekerli robot için R ve L değerlerini sormuş ama 
# A* algoritması grid bazlı olduğu için kinematik denklemlere bağlayamadım.
# Şimdilik sadece A* kısmını nokta robot gibi modelledim.
R = 0.1
L = 0.5

# 50x50 harita matrisi
harita = np.zeros((50, 50))

# engelleri manuel ekliyoruz
harita[15:35, 20:25] = 1  # ortadaki uzun duvar
harita[10:15, 10:15] = 1  # diğer ufak engel

baslangic = (5, 5)
hedef = (45, 40)

# Kuş uçuşu mesafe hesabı (öklid)
def mesafe_hesapla(dugum, hedef):
    return math.sqrt((dugum[0] - hedef[0])**2 + (dugum[1] - hedef[1])**2)

def a_star_coz(grid, start, goal):
    # 8 yöne hareket edebiliyor
    hareketler = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    
    open_list = [start]
    closed_list = set()
    
    g_degerleri = {start: 0}
    f_degerleri = {start: mesafe_hesapla(start, goal)}
    gelinen_yer = {} # en son rotayı çizmek için ebeveynleri tutucaz
    
    while len(open_list) > 0:
        # open_list içindeki en küçük f değerine sahip düğümü bul
        current = open_list[0]
        for dugum in open_list:
            if f_degerleri.get(dugum, 999999) < f_degerleri.get(current, 999999):
                current = dugum
                
        # hedefe ulaştık mı kontrolü
        if current == goal:
            # rotayı sondan başa doğru geri sar
            rota = []
            while current in gelinen_yer:
                rota.append(current)
                current = gelinen_yer[current]
            rota.append(start)
            rota.reverse()
            return rota
            
        open_list.remove(current)
        closed_list.add(current)
        
        for h in hareketler:
            komsu = (current[0] + h[0], current[1] + h[1])
            
            # harita sınırlarından çıktı mı?
            if komsu[0] < 0 or komsu[0] >= 50 or komsu[1] < 0 or komsu[1] >= 50:
                continue
                
            # engele mi çarptık?
            if grid[komsu[0]][komsu[1]] == 1:
                continue
                
            if komsu in closed_list:
                continue
                
            # çapraz gidiyorsa kök 2, düz gidiyorsa 1 maliyet ekle
            adim_maliyeti = math.sqrt(h[0]**2 + h[1]**2)
            yeni_g = g_degerleri[current] + adim_maliyeti
            
            if komsu not in open_list:
                open_list.append(komsu)
            elif yeni_g >= g_degerleri.get(komsu, 999999):
                continue # daha iyi bir yol değilse geç
                
            # daha kısa yol bulduk, kaydet
            gelinen_yer[komsu] = current
            g_degerleri[komsu] = yeni_g
            f_degerleri[komsu] = g_degerleri[komsu] + mesafe_hesapla(komsu, goal)
            
    return None # döngü biterse ve hedef bulunamazsa

# ----- ANA KOD KISMI -----

bulunan_rota = a_star_coz(harita, baslangic, hedef)

if bulunan_rota is None:
    print("Hedefe giden yol bulunamadı!")
else:
    # print("Yol bulundu:", bulunan_rota) # debug için yazmıştım
    
    rota_y = []
    rota_x = []
    for nokta in bulunan_rota:
        rota_y.append(nokta[0])
        rota_x.append(nokta[1])

    # Görselleştirme
    fig, ax = plt.subplots()
    ax.imshow(harita, cmap='gray_r') # siyah engeller, beyaz zemin
    
    cizgi, = ax.plot([], [], 'b-', linewidth=2) # mavi çizgi
    robot_noktasi, = ax.plot([], [], 'ro', markersize=6) # kırmızı robot
    
    ax.plot(baslangic[1], baslangic[0], 'go', markersize=8, label="Başlangıç")
    ax.plot(hedef[1], hedef[0], 'mo', markersize=8, label="Hedef")
    
    ax.legend()
    plt.title("A* Yol Planlama Ödevi")
    
    def animasyon_guncelle(i):
        cizgi.set_data(rota_x[:i+1], rota_y[:i+1])
        robot_noktasi.set_data([rota_x[i]], [rota_y[i]])
        return cizgi, robot_noktasi

    ani = animation.FuncAnimation(fig, animasyon_guncelle, frames=len(bulunan_rota), interval=100)
    plt.show()