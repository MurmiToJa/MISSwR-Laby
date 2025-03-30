import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, Scale
from PIL import Image, ImageDraw
import numpy as np

class BinaryMapDrawer:
    def __init__(self, root):
        self.root = root
        self.root.title("Binary Map Drawer - Siatka")
        
        # Domyślne wartości
        self.width = 800
        self.height = 600
        self.resolution = 10  # pikseli na metr
        self.grid_size = 10   # rozmiar komórki siatki (taki sam jak rozdzielczość)
        self.drawing = False
        self.current_value = 1  # 1 = przeszkoda (czarny), 0 = wolna przestrzeń (biały)
        self.zoom_factor = 1.0  # domyślny współczynnik przybliżenia
        
        # Pobierz wymiary i rozdzielczość od użytkownika przy starcie
        self.get_initial_parameters()
        
        # Obliczenie liczby komórek siatki
        self.grid_width = self.width // self.grid_size
        self.grid_height = self.height // self.grid_size
        
        # Tworzenie pustej mapy (biała = wolna przestrzeń)
        self.map_array = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)
        
        # Zmienne do obsługi przewijania
        self.scroll_x = 0
        self.scroll_y = 0
        
        # Tworzenie interfejsu
        self.create_widgets()
        
    def get_initial_parameters(self):
        # Pytanie o wymiary mapy
        width = simpledialog.askinteger("Szerokość mapy", "Podaj szerokość mapy w pikselach:", 
                                        initialvalue=10, minvalue=10, maxvalue=2000)
        height = simpledialog.askinteger("Wysokość mapy", "Podaj wysokość mapy w pikselach:", 
                                         initialvalue=10, minvalue=10, maxvalue=2000)
        
        # Pytanie o rozdzielczość
        resolution = simpledialog.askinteger("Rozdzielczość", "Podaj rozdzielczość (pikseli na metr):", 
                                             initialvalue=10, minvalue=1, maxvalue=100)
        
        if width is not None:
            self.width = width
        if height is not None:
            self.height = height
        if resolution is not None:
            self.resolution = resolution
            self.grid_size = resolution
    
    def create_widgets(self):
        # Górny panel z przyciskami
        self.control_frame = tk.Frame(self.root)
        self.control_frame.pack(side=tk.TOP, fill=tk.X)
        
        # Przycisk do rysowania przeszkód (czarny)
        self.obstacle_button = tk.Button(self.control_frame, text="Rysuj Przeszkody", 
                                         command=lambda: self.set_drawing_mode(1), bg="black", fg="white")
        self.obstacle_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Przycisk do rysowania wolnej przestrzeni (biały)
        self.free_button = tk.Button(self.control_frame, text="Rysuj Wolną Przestrzeń", 
                                     command=lambda: self.set_drawing_mode(0), bg="white", fg="black")
        self.free_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Przycisk do czyszczenia mapy
        self.clear_button = tk.Button(self.control_frame, text="Wyczyść Mapę", command=self.clear_map)
        self.clear_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Przycisk do zapisywania mapy
        self.save_button = tk.Button(self.control_frame, text="Zapisz jako JPG", command=self.save_map)
        self.save_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Przycisk do exportu jako macierz NumPy
        self.export_button = tk.Button(self.control_frame, text="Export do NumPy", command=self.export_numpy)
        self.export_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Przycisk do pokazania/ukrycia siatki
        self.grid_button = tk.Button(self.control_frame, text="Pokaż/Ukryj Siatkę", command=self.toggle_grid)
        self.grid_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Panel kontroli przybliżenia
        self.zoom_frame = tk.Frame(self.control_frame)
        self.zoom_frame.pack(side=tk.LEFT, padx=5, pady=5)
        
        self.zoom_label = tk.Label(self.zoom_frame, text="Przybliżenie:")
        self.zoom_label.pack(side=tk.LEFT)
        
        self.zoom_scale = Scale(self.zoom_frame, from_=1, to=10, orient=tk.HORIZONTAL, 
                               resolution=0.5, length=150, command=self.update_zoom)
        self.zoom_scale.set(1.0)
        self.zoom_scale.pack(side=tk.LEFT)
        
        # Przycisk do resetowania widoku
        self.reset_view_button = tk.Button(self.control_frame, text="Resetuj Widok", command=self.reset_view)
        self.reset_view_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Informacja o rozdzielczości
        self.info_label = tk.Label(self.control_frame, 
                                  text=f"Rozdzielczość: {self.resolution} px/m, Rozmiar siatki: {self.grid_width}x{self.grid_height} komórek")
        self.info_label.pack(side=tk.RIGHT, padx=5, pady=5)
        
        # Ramka z suwakami do przewijania (dla dużych map)
        self.scroll_frame = tk.Frame(self.root)
        self.scroll_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Canvas do rysowania z paskami przewijania
        self.canvas_frame = tk.Frame(self.root)
        self.canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        # Paski przewijania
        self.h_scrollbar = tk.Scrollbar(self.canvas_frame, orient=tk.HORIZONTAL)
        self.h_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.v_scrollbar = tk.Scrollbar(self.canvas_frame)
        self.v_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Canvas do rysowania
        self.canvas = tk.Canvas(self.canvas_frame, width=self.width, height=self.height, bg="white",
                               xscrollcommand=self.h_scrollbar.set, yscrollcommand=self.v_scrollbar.set)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Konfiguracja pasków przewijania
        self.h_scrollbar.config(command=self.canvas.xview)
        self.v_scrollbar.config(command=self.canvas.yview)
        
        # Tworzenie ramki wewnątrz Canvas, która będzie przybliżana
        self.canvas_content = tk.Frame(self.canvas)
        self.canvas.create_window((0, 0), window=self.canvas_content, anchor="nw")
        
        # Podłączenie zdarzeń myszy
        self.canvas.bind("<Button-1>", self.start_drawing)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.stop_drawing)
        self.canvas.bind("<MouseWheel>", self.mouse_zoom)  # Windows
        self.canvas.bind("<Button-4>", self.mouse_zoom)    # Linux scroll up
        self.canvas.bind("<Button-5>", self.mouse_zoom)    # Linux scroll down
        
        # Zmienna do przechowywania linii siatki
        self.grid_lines = []
        self.show_grid = True
        
        # Aktualizacja regionu przewijania
        self.update_scroll_region()
        self.draw_grid()
    
    def update_zoom(self, event=None):
        # Pobierz aktualną wartość przybliżenia z suwaka
        new_zoom = float(self.zoom_scale.get())
        if new_zoom != self.zoom_factor:
            # Zapisz aktualną pozycję widoku
            center_x = self.canvas.canvasx(self.canvas.winfo_width() // 2)
            center_y = self.canvas.canvasy(self.canvas.winfo_height() // 2)
            
            self.zoom_factor = new_zoom
            
            # Zaktualizuj region przewijania i przerysuj wszystko
            self.redraw_canvas()
            self.update_scroll_region()
            
            # Przesunięcie do poprzedniego centrum
            self.canvas.xview_moveto((center_x * new_zoom) / self.canvas.winfo_width())
            self.canvas.yview_moveto((center_y * new_zoom) / self.canvas.winfo_height())
    
    def mouse_zoom(self, event):
        # Obsługa przybliżania kółkiem myszy
        if event.num == 4 or event.delta > 0:  # Przybliżenie
            new_zoom = min(self.zoom_factor + 0.5, 10.0)
        elif event.num == 5 or event.delta < 0:  # Oddalenie
            new_zoom = max(self.zoom_factor - 0.5, 1.0)
        else:
            return
        
        # Aktualizuj suwak i wywołaj funkcję przybliżenia
        self.zoom_scale.set(new_zoom)
        self.update_zoom()
    
    def redraw_canvas(self):
        # Czyszczenie canvas
        self.canvas.delete("all")
        
        # Rysowanie wszystkich komórek
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                x1 = x * self.grid_size * self.zoom_factor
                y1 = y * self.grid_size * self.zoom_factor
                x2 = x1 + self.grid_size * self.zoom_factor
                y2 = y1 + self.grid_size * self.zoom_factor
                
                if self.map_array[y, x] == 1:  # przeszkoda (czarny)
                    color = "black"
                else:  # wolna przestrzeń (biały)
                    color = "white"
                
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="", 
                                           tags=(f"cell_{x}_{y}"))
        
        # Przerysowanie siatki jeśli jest włączona
        if self.show_grid:
            self.draw_grid()
    
    def update_scroll_region(self):
        # Obliczanie rozmiaru obszaru przewijania na podstawie przybliżenia
        scroll_width = self.width * self.zoom_factor
        scroll_height = self.height * self.zoom_factor
        self.canvas.config(scrollregion=(0, 0, scroll_width, scroll_height))
    
    def reset_view(self):
        # Resetowanie przybliżenia i przewijania
        self.zoom_scale.set(1.0)
        self.update_zoom()
        self.canvas.xview_moveto(0)
        self.canvas.yview_moveto(0)
    
    def toggle_grid(self):
        self.show_grid = not self.show_grid
        if self.show_grid:
            self.draw_grid()
        else:
            self.clear_grid()
    
    def draw_grid(self):
        # Czyszczenie poprzednich linii siatki
        self.clear_grid()
        
        # Oblicz rozmiar przybliżonej komórki siatki
        grid_size_zoomed = self.grid_size * self.zoom_factor
        
        # Rysowanie linii poziomych
        for i in range(0, int(self.height * self.zoom_factor) + 1, int(grid_size_zoomed)):
            line = self.canvas.create_line(0, i, self.width * self.zoom_factor, i, fill="#DDDDDD")
            self.grid_lines.append(line)
        
        # Rysowanie linii pionowych
        for i in range(0, int(self.width * self.zoom_factor) + 1, int(grid_size_zoomed)):
            line = self.canvas.create_line(i, 0, i, self.height * self.zoom_factor, fill="#DDDDDD")
            self.grid_lines.append(line)
    
    def clear_grid(self):
        for line in self.grid_lines:
            self.canvas.delete(line)
        self.grid_lines = []
    
    def set_drawing_mode(self, value):
        self.current_value = value
    
    def start_drawing(self, event):
        self.drawing = True
        self.draw(event)
    
    def draw(self, event):
        if self.drawing:
            # Pobieranie aktualnych współrzędnych Canvas (uwzględniając przewijanie)
            canvas_x = self.canvas.canvasx(event.x)
            canvas_y = self.canvas.canvasy(event.y)
            
            # Obliczanie indeksu komórki siatki na podstawie współrzędnych myszy i przybliżenia
            grid_x = int(canvas_x / (self.grid_size * self.zoom_factor))
            grid_y = int(canvas_y / (self.grid_size * self.zoom_factor))
            
            # Upewnienie się, że współrzędne są w granicach mapy
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                # Aktualizacja wartości w macierzy
                self.map_array[grid_y, grid_x] = self.current_value
                
                # Obliczanie współrzędnych komórki siatki
                x1 = grid_x * self.grid_size * self.zoom_factor
                y1 = grid_y * self.grid_size * self.zoom_factor
                x2 = x1 + self.grid_size * self.zoom_factor
                y2 = y1 + self.grid_size * self.zoom_factor
                
                # Usunięcie istniejącego prostokąta jeśli istnieje
                self.canvas.delete(f"cell_{grid_x}_{grid_y}")
                
                # Rysowanie wypełnionego prostokąta
                if self.current_value == 1:  # przeszkoda (czarny)
                    color = "black"
                else:  # wolna przestrzeń (biały)
                    color = "white"
                
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="", 
                                           tags=(f"cell_{grid_x}_{grid_y}"))
                
                # Przerysowanie siatki jeśli jest włączona
                if self.show_grid:
                    # Rysowanie krawędzi komórki
                    if self.current_value == 0:  # biała komórka, rysuj szare linie
                        self.canvas.create_line(x1, y1, x2, y1, fill="#DDDDDD")
                        self.canvas.create_line(x1, y1, x1, y2, fill="#DDDDDD")
                        self.canvas.create_line(x2, y1, x2, y2, fill="#DDDDDD")
                        self.canvas.create_line(x1, y2, x2, y2, fill="#DDDDDD")
    
    def stop_drawing(self, event):
        self.drawing = False
    
    def clear_map(self):
        # Resetowanie macierzy mapy
        self.map_array = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)
        
        # Czyszczenie canvas i przerysowanie
        self.redraw_canvas()
    
    def save_map(self):
        # Tworzenie obraz o wymiarach siatki
        img = Image.new('RGB', (self.grid_width, self.grid_height), color='white')
        draw = ImageDraw.Draw(img)
        
        # Wypełnianie obrazu na podstawie macierzy
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if self.map_array[y, x] == 1:
                    draw.point((x, y), fill='black')
                else:
                    draw.point((x, y), fill='white')
        
        # Powiększanie obrazu do oryginalnych wymiarów
        img_resized = img.resize((self.width, self.height), Image.NEAREST)
        
        # Dialog zapisu pliku
        file_path = filedialog.asksaveasfilename(defaultextension=".jpg", 
                                                filetypes=[("JPEG files", "*.jpg"), ("All files", "*.*")])
        
        if file_path:
            img_resized.save(file_path)
            messagebox.showinfo("Sukces", f"Mapa została zapisana do {file_path}")
            
            # Dodanie informacji o metadanych
            with open(file_path + ".info.txt", "w") as f:
                f.write(f"Resolution: {self.resolution} pixels per meter\n")
                f.write(f"Grid size: {self.grid_size} pixels\n")
                f.write(f"Grid dimensions: {self.grid_width}x{self.grid_height} cells\n")
                f.write(f"Image dimensions: {self.width}x{self.height} pixels\n")
                f.write("Format: Binary map (0=free space, 1=obstacle)\n")
    
    def export_numpy(self):
        # Dialog zapisu pliku
        file_path = filedialog.asksaveasfilename(defaultextension=".npy", 
                                               filetypes=[("NumPy files", "*.npy"), ("All files", "*.*")])
        
        if file_path:
            np.save(file_path, self.map_array)
            messagebox.showinfo("Sukces", f"Mapa została zapisana jako macierz NumPy do {file_path}")
            
            # Dodanie informacji o metadanych
            with open(file_path + ".info.txt", "w") as f:
                f.write(f"Resolution: {self.resolution} pixels per meter\n")
                f.write(f"Grid dimensions: {self.grid_width}x{self.grid_height} cells\n")
                f.write(f"Original dimensions: {self.width}x{self.height} pixels\n")
                f.write("Format: NumPy binary array (0=free space, 1=obstacle)\n")

# Uruchomienie aplikacji
if __name__ == "__main__":
    root = tk.Tk()
    app = BinaryMapDrawer(root)
    root.mainloop()