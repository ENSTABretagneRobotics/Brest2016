##########################################################################
# Behavior MANAGER
##########################################################################
from behavior import Behavior


class BehaviorManager():
    """Cette classe contient les outils pour creer,
     mettre a jour et publier les listes d'obstacles
     et d'objectifs, ainsi que les champs de vecteurs
     associes """

    def __init__(self, sailboat=False):
        self.behavior_list = []
        self.champ_total = Behavior()
        self.sailboat = sailboat

    def handle_behavior(self, new_behavior, mode):
        """
        Handles a new behavior received according to the mode:
        :mode == add
            add the behavior only if NOT on the list. else do nothing
        :mode == update
            add the behavior if not on the list
            udpate the behavior if on the list
        :mode == remove
            remove the behavior from the list if in
        """
        what_was_done = 'nothing'
        if mode == 'add' and new_behavior not in self.behavior_list:
            self.behavior_list.append(new_behavior)
            self.champ_total += new_behavior
            what_was_done = 'added'
        elif mode == 'update':
            if new_behavior not in self.behavior_list:
                self.behavior_list.append(new_behavior)
                self.champ_total += new_behavior
                what_was_done = 'added'
            else:
                i = self.behavior_list.index(new_behavior)
                self.behavior_list[i] = new_behavior
                what_was_done = 'updated'
        elif mode == 'remove' and new_behavior in self.behavior_list:
            self.behavior_list.remove(new_behavior)
            what_was_done = 'removed'
        elif mode == 'clear_all':
            self.behavior_list = []
            what_was_done = 'cleared all'

        self.recalc_champ_total()
        return what_was_done

    def recalc_champ_total(self):
        self.champ_total = Behavior()
        for c in self.behavior_list:
            self.champ_total += c
        if self.sailboat:
            self.champ_total.projection = True

    def getState(self):
        pass
