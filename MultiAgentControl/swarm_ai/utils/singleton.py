# -*- coding: utf-8 -*-
# Signleton class for use in building dynamic objects
# that only need one instance at a time.
#
# Created By: Tyler Fedrizzi
# Initial Date: 3/27/2020
#
# Copyright Codex Laboratories LLC

class Singleton:

    def __init__(self, cls):
        self.__class__ = cls

    def Instance(self):
        try:
            return self._instance
        except AttributeError:
            self._instance = self._cls()
            return self._instance



    def __call__(self):
        raise TypeError('Singletons must be accessed through `Instance()`.')



    def __instancecheck__(self, inst):
        return isinstance(inst, self._cls)