class Tag:
    def __init__(self, canal, ID, x, y, z, pqf):
        self.canal = canal
        self.ID = ID
        self.x = x
        self.y = y
        self.z = z
        self.pqf = pqf

    def atualizar_dados(self, x, y, z, pqf):
        self.x = x
        self.y = y
        self.z = z
        self.pqf = pqf

    def __str__(self):
        return f"Tag(ID={self.ID}, Canal={self.canal}, x={self.x}, y={self.y}, z={self.z}, pqf={self.pqf})"
