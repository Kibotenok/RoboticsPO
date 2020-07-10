#!/usr/bin/env python3

class PID:

    def __init__(self, kp, ki, kd):
        """ PID regulator
            :param kp: proportional coefficient
            :param ki: integral coefficient
            :param kd: differential coefficient
        """
        self._kp = kp
        self._ki = ki
        self._kd = kd

        self._prev_i = 0
        self._prev_err = 0

    def _p_calc(self, err):
        """ Calculate proportional part
            :param err: error value

            :return proportional part value
        """
        return self._kp*err

    def _i_calc(self, err):
        """ Calculate integral part
            :param err: error value

            :return integral part value
        """
        return self._prev_i + self._ki*err

    def _d_calc(self, err):
        """ Calculate differential part
            :param err: error value

            :return differential part value
        """
        return self._kd*(err - self._prev_err)

    def update(self):
        """ Update errors value"""
        self._prev_err = 0
        self._prev_i = 0

    def regulator(self, target):
        """ Calculate new value based on pid regulator
            :param target: target value

            :return new value
        """
        p_part = self._p_calc(target)
        i_part = self._i_calc(target)
        d_part = self._d_calc(target)

        self._prev_i = i_part
        self._prev_err = target

        return p_part+i_part+d_part

