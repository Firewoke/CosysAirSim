import numpy as np

def read_binvox(path):
    with open(path, "rb") as f:
        line = f.readline().strip()
        if line != b"#binvox 1":
            raise ValueError("Not a binvox file or unsupported version")

        dims = None
        translate = None
        scale = None

        # header
        while True:
            line = f.readline().strip()
            if line.startswith(b"dim"):
                dims = list(map(int, line.split()[1:]))
            elif line.startswith(b"translate"):
                translate = list(map(float, line.split()[1:]))
            elif line.startswith(b"scale"):
                scale = float(line.split()[1])
            elif line == b"data":
                break

        if dims is None:
            raise ValueError("Missing dim in header")

        # data (run-length encoding)
        size = dims[0] * dims[1] * dims[2]
        values = np.empty(size, dtype=np.uint8)

        idx = 0
        while idx < size:
            value = ord(f.read(1))
            count = ord(f.read(1))
            values[idx:idx+count] = value
            idx += count

        grid = values.reshape((dims[0], dims[1], dims[2]), order="C")
        # binvox convention is sometimes xzy; we keep raw. We'll just use it consistently.
        return grid, dims, translate, scale


if __name__ == "__main__":
    path = r"D:\MyProject\DroneProject_py\voxel_Drone1_5x5x20_res0.25_20260304_184917.binvox"
    grid, dims, translate, scale = read_binvox(path)

    occ = int(grid.sum())
    total = grid.size
    print("dims:", dims, "total:", total, "occupied:", occ, "occ%:", occ/total*100)

    # Print a middle slice (rough visualization)
    z = dims[2] // 2
    sl = grid[:, :, z]
    # downsample printing
    print("middle z slice (1=occ, 0=free):")
    for y in range(sl.shape[1]):
        row = "".join("#" if sl[x, y] else "." for x in range(sl.shape[0]))
        print(row)