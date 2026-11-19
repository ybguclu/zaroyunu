# 🎲 Vanilla C++ Win32 - 3D Tavla Zarı Oyun Motoru

Hiçbir harici kütüphane veya 3D grafik API'si (**DirectX, OpenGL, Vulkan, SDL, SFML YOK!**) kullanılmadan, **%100 Saf (Vanilla) C++17 ve Win32 API** ile sıfırdan yazılmış **3D Yazılımsal Oyun ve Fizik Motoru (Software 3D Renderer & Rigid-Body Physics)**.

## 🚀 Motorun Teknik Özellikleri

- **Sıfırdan 3D Yazılımsal Rasterizer (CPU):** Perspektif düzeltmeli doku haritalama (Perspective-Correct UV Mapping), **Z-Buffer (Derinlik Tamponu)**, Backface Culling ve Lambertian + Specular ışıklandırma.
- **Raytraced 3D Ahşap Tavla Kutusu:** Işın-Kutu kesişimi (`Ray-AABB Intersection`) ile gerçek 3D yükseltilmiş ahşap kenarlar, pirinç menteşeler, duvarların tahtaya düşen gerçek gölgeleri ve önceden pişirilmiş derinlik haritası (`backgroundZBuffer`).
- **Gerçek 3D Katı Cisim (Rigid-Body) Zar Fiziği:**
  - **Quaternion** tabanlı 3D dönüş (Gimbal Lock yok).
  - 8 köşe zemin temas algılaması ve **Impulse (İtki) tabanlı Coulomb yuvarlanma sürtünmesi** (zarlar kaymaz, köşeleri üzerinde takla atarak yuvarlanır).
  - Hilesiz zar okuma: Sonuç rastgele bir sayıdan değil, fiziksel yuvarlanma bittiğinde gerçekte üstte kalan yüzey normalinden (`+Y`) okunur.
- **Prosedürel Doku Üretimi:** Hiçbir harici görsel dosyası (`.png`/`.bmp`) kullanmadan kod içinde üretilen ahşap damarları ve fildişi zar yüzeyi dokuları.
- **Win32 Çift Tamponlu (Zero-Flicker) Çizim & MCI Ses Motoru:** `CreateDIBSection` + `BitBlt` ile titreşimsiz 60+ FPS çizim ve `winmm.lib` (MCI) ile zarların ahşaba ilk temas anına senkronlu `.mp3` ses efekti.

## 🎮 Kontroller

- **`BOŞLUK (SPACE)` / `ENTER` / `Sol Tık`:** Zarları fırlat
- **`R`:** Skoru sıfırla
- **`ESC`:** Çıkış

## 🛠️ Derleme

### Visual Studio 2022
Projeyi açıp `x64 - Release` modunda doğrudan derleyip çalıştırabilirsiniz (`C++17` standardı kullanılır).

### MinGW (GCC)
```bash
g++ -O2 -std=c++17 main.cpp -o tavla_zari.exe -lgdi32 -luser32 -lwinmm -mwindows