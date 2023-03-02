import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt


class FuzzyLogic:
    def __init__(self, a):
        if a == "naikturun":

            # self.angle = ctrl.Antecedent(np.arange())
            self.angle = ctrl.Antecedent(np.arange(-35, 30, .1), 'ori_x')
            self.velocity = ctrl.Antecedent(np.arange(0, 7, .1), 'gyro_x')
            self.step = ctrl.Consequent(np.arange(0.1, 0.4, .01), 'Q')

            self.angle['Thigh'] = fuzz.trimf(self.angle.universe, [-35, -35, -25])
            self.angle['Tmedium'] = fuzz.trimf(self.angle.universe, [-30, -23, -15])
            self.angle['Tlow'] = fuzz.trimf(self.angle.universe, [-17, -10, 0])

            # self.angle['Nlow'] = fuzz.trimf(self.angle.universe, [-17, -10, 0])
            # self.angle['Nmedium'] = fuzz.trimf(self.angle.universe, [-5,5,10 ])
            # self.angle['Nhigh'] = fuzz.trimf(self.angle.universe, [5, 10, 10])
            
            self.angle['Nlow'] = fuzz.trimf(self.angle.universe, [-17, -10, 0])
            self.angle['Nmedium'] = fuzz.trimf(self.angle.universe, [-5,10,25 ])
            self.angle['Nhigh'] = fuzz.trimf(self.angle.universe, [15, 30, 30])

            self.velocity['low'] = fuzz.trimf(self.velocity.universe, [0, 0, 5])
            # self.velocity['medium'] = fuzz.trimf(self.velocity.universe, [1.2, 6, 10.8])
            self.velocity['high'] = fuzz.trimf(self.velocity.universe, [3, 7, 7])

            # self.step['Nmedium'] = fuzz.trimf(self.step.universe, [0.23, 0.28, 0.35])
            # self.step['Nhigh'] = fuzz.trimf(self.step.universe, [0.33, 0.4, 0.4])
            self.step['Nmedium'] = fuzz.trimf(self.step.universe, [0.23, 0.3, 0.37])
            self.step['Nhigh'] = fuzz.trimf(self.step.universe, [0.33, 0.4, 0.4])

            self.step['Thigh'] = fuzz.trimf(self.step.universe, [0.1, 0.1, 0.15])
            self.step['Tmedium'] = fuzz.trimf(self.step.universe, [0.13, 0.18, 0.23])
            self.step['NTlow'] = fuzz.trimf(self.step.universe, [0.2, 0.23, 0.25])

            # # self.angle = ctrl.Antecedent(np.arange())
            # self.angleY = ctrl.Antecedent(np.arange(-5, 5, .1), 'ori_y')
            # # self.velocity = ctrl.Antecedent(np.arange(0, 27, .1), 'gyro_x')
            # self.stepY = ctrl.Consequent(np.arange(0.1, 0.36, .01), 'Q')
            #
            # self.angleY['Kahigh'] = fuzz.trimf(self.angleY.universe, [-5, -5, -3])
            # self.angleY['Kamedium'] = fuzz.trimf(self.angleY.universe, [-4, -2, 0])
            # self.angleY['Kalow'] = fuzz.trimf(self.angleY.universe, [-1, 0, 1])
            #
            # self.angleY['Kilow'] = fuzz.trimf(self.angleY.universe, [-1, 0, 1])
            # self.angleY['Kimedium'] = fuzz.trimf(self.angleY.universe, [0, 2, 4])
            # self.angleY['Kihigh'] = fuzz.trimf(self.angleY.universe, [3, 5, 5])
            #
            # # self.velocity['low'] = fuzz.trimf(self.velocity.universe, [0, 0, 17])
            # # # self.velocity['medium'] = fuzz.trimf(self.velocity.universe, [1.2, 6, 10.8])
            # # self.velocity['high'] = fuzz.trimf(self.velocity.universe, [13, 27, 27])
            #
            # self.stepY['Kamedium'] = fuzz.trimf(self.stepY.universe, [0.23, 0.28, 0.33])
            # self.stepY['Kahigh'] = fuzz.trimf(self.stepY.universe, [0.3, 0.35, 0.35])
            #
            # self.stepY['Kihigh'] = fuzz.trimf(self.stepY.universe, [0.1, 0.1, 0.15])
            # self.stepY['Kimedium'] = fuzz.trimf(self.stepY.universe, [0.13, 0.18, 0.23])
            # self.stepY['Klow'] = fuzz.trimf(self.stepY.universe, [0.2, 0.23, 0.25])

        rule1 = ctrl.Rule((self.angle['Nlow'] or self.angle['Tlow']) & self.velocity['low'], self.step['NTlow'])
        rule2 = ctrl.Rule(self.angle['Nmedium'] & self.velocity['low'], self.step['Nmedium'])
        rule3 = ctrl.Rule(self.angle['Nhigh'] & self.velocity['low'], self.step['Nhigh'])

        rule4 = ctrl.Rule(self.angle['Nlow'] & self.velocity['high'], self.step['Nmedium'])
        rule5 = ctrl.Rule(self.angle['Nmedium'] & self.velocity['high'], self.step['Nhigh'])
        rule6 = ctrl.Rule(self.angle['Nhigh'] & self.velocity['high'], self.step['Nhigh'])

        rule7 = ctrl.Rule(self.angle['Tmedium'] & self.velocity['low'], self.step['Tmedium'])
        rule8 = ctrl.Rule(self.angle['Thigh'] & self.velocity['low'], self.step['Tmedium'])

        rule9 = ctrl.Rule(self.angle['Tlow'] & self.velocity['high'], self.step['Tmedium'])
        rule10 = ctrl.Rule(self.angle['Tmedium'] & self.velocity['high'], self.step['Thigh'])
        rule11 = ctrl.Rule(self.angle['Thigh'] & self.velocity['high'], self.step['Thigh'])

        # rule12 = ctrl.Rule((self.angleY['Kalow'] or self.angleY['Kilow']) & self.velocity['low'], self.stepY['Klow'])
        # rule13 = ctrl.Rule(self.angleY['Kamedium'] & self.velocity['low'], self.stepY['Kamedium'])
        # rule14 = ctrl.Rule(self.angleY['Kahigh'] & self.velocity['low'], self.stepY['Kamedium'])
        #
        # rule15 = ctrl.Rule(self.angleY['Kalow'] & self.velocity['high'], self.stepY['Kamedium'])
        # rule5 = ctrl.Rule(self.angle['Kamedium'] & self.velocity['high'], self.step['Kahigh'])
        # rule6 = ctrl.Rule(self.angle['Kahigh'] & self.velocity['high'], self.step['Kahigh'])
        #
        # rule7 = ctrl.Rule(self.angle['Kimedium'] & self.velocity['low'], self.step['Kimedium'])
        # rule8 = ctrl.Rule(self.angle['Kihigh'] & self.velocity['low'], self.step['Kimedium'])
        #
        # rule9 = ctrl.Rule(self.angle['Kilow'] & self.velocity['high'], self.step['Kimedium'])
        # rule10 = ctrl.Rule(self.angle['Kimedium'] & self.velocity['high'], self.step['Kihigh'])
        # rule11 = ctrl.Rule(self.angle['Kihigh'] & self.velocity['high'], self.step['Kihigh'])

        # rule7 = ctrl.Rule(self.angle['low'] & self.velocity['high'], self.step['low'])
        # rule8 = ctrl.Rule(self.angle['medium'] & self.velocity['high'], self.step['medium'])
        # rule9 = ctrl.Rule(self.angle['high'] & self.velocity['high'], self.step['medium'])

        self.step_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11])#, rule12, rule13, rule14, rule15])
        self.steping = ctrl.ControlSystemSimulation(self.step_ctrl)

    def compute(self,angleX, angleY, velocity):
        # self.steping.input['ori_y'] = angleY
        self.steping.input['ori_x'] = angleX

        self.steping.input['gyro_x'] = velocity

        # Crunch the numbers
        self.steping.compute()

        return self.steping.output['Q']

    def vizualize(self):

        self.angle.view()
        self.velocity.view()
        self.step.view()
        self.step.view(sim=self.steping)
        # self.stepY.view(sim=self.steping)
        plt.pause(100)


def main():
    global a,c
    # c=0
    # for z in range(200):
    #     if c % 4 == 0:
    #         print(c)
    a = "naikturun"
    fz = FuzzyLogic(a)

        #     fz.angle.view()
    result = fz.compute(22,0, 1)
    print(result)

        # c=c+1
    # fz.vizualize()


if __name__ == "__main__":
    main()