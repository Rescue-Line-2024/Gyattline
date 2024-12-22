from ric_colori import RiconosciColori

class GestioneVerdi(RiconosciColori):
    def __init__(self, bottom_value, upper_value,min_area = 100):
        super().__init__(bottom_value, upper_value)
        self.min_area = min_area

    def riconosci_verdi(self, image):
        """
        Riconosce il verde nell'immagine e ritorna le coordinate dei bounding box.
        :param image: L'immagine in ingresso.
        :return: Una lista di bounding box dei contorni verdi.
        """
        return self.riconosci_colore(image, self.min_area)

    