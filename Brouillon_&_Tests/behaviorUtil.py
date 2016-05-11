#!/usr/bin/env  python


class BehaviorManager():
    """Cette classe contient les outils pour créer,
     mettre à jour et publier les listes d'obstacles
     et d'objectifs, ainsi que les champs de vecteurs
     associés """

    def __init__(self):
        self.behavior_list = []
        self.field_list = []
        self.vectorField = 0

    def check_list(self):
        for behavior in self.behavior_list:
            pass

    def toFields(self):
        k = 0
        for behavior in self.behavior_list:
            self.field_list[k] = self.createField(behavior)
            k += 1

    def sumFields(self):
        for field in self.field_list:
            pass 
        # vectorfield = somme de tous

    def createField(behavior):
        pass

