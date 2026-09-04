import numpy as np
from PIL import Image
import struct
import math
from pathlib import Path


def decode_lora_fixed(file_path: Path) -> Image.Image:
    with file_path.open("rb") as f:
        # Lê W e H (Little Endian)
        header = f.read(4)
        if len(header) != 4:
            raise ValueError("Arquivo muito pequeno")

        w, h = struct.unpack("<HH", header)
        print(f"  Resolucao: {w}x{h}")

        # 2 pixels por byte (4bpp)
        bytes_per_line = math.ceil(w / 2)

        pixels = []
        for _ in range(h):
            line_data = f.read(bytes_per_line)
            if len(line_data) != bytes_per_line:
                raise ValueError("EOF inesperado ao ler linha")

            line_pixels = []
            for byte in line_data:
                p1 = (byte >> 4) & 0x0F
                p2 = byte & 0x0F
                line_pixels.append(p1 * 17)  # expande 0..15 → 0..255
                line_pixels.append(p2 * 17)

            pixels.extend(line_pixels[:w])

        img_array = np.array(pixels, dtype=np.uint8).reshape((h, w))
        return Image.fromarray(img_array, mode="L")


def process_directory(input_dir: str | Path):
    input_dir = Path(input_dir)

    if not input_dir.is_dir():
        raise ValueError(f"Não é um diretório válido: {input_dir}")

    bin_files = sorted(input_dir.glob("*.bin"))
    if not bin_files:
        print("Nenhum arquivo .bin encontrado.")
        return

    print(f"Processando {len(bin_files)} arquivos em {input_dir}\n")

    for bin_path in bin_files:
        print(f"Arquivo: {bin_path.name}")
        try:
            img = decode_lora_fixed(bin_path)
            out_name = bin_path.stem + ".png"
            out_path = Path.cwd() / out_name
            img.save(out_path)
            print(f"  -> salvo como {out_path.name}\n")
        except Exception as e:
            print(f"  ERRO ao processar {bin_path.name}: {e}\n")


if __name__ == "__main__":
    # exemplo:
    # process_directory("THUMB")
    process_directory("THUMB")

