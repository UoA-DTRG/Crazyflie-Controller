import time
import cflib.crtp# type: ignore
from cflib.crazyflie.swarm import CachedCfFactory# type: ignore
from cflib.crazyflie.swarm import Swarm# type: ignore
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import syncCrazyflie# type: ignore
from udp_client import UDP_Client
import traceback
import math 
import matplotlib.pyplot as plt
from vicon_connection_class import ViconInterface as vi
from timed_queue import TimedQueue
import cProfile
import pstats
import logging
from logging.handlers import RotatingFileHandler
from queue import Queue
import threading
from collections import namedtuple
import numpy as np
import pygame


# OBJECT NAMES
OBJECT_NAME = "CrazyfliePayload"
LEFT_DRONE_NAME = "PbodyCrazyFlie"
RIGHT_DRONE_NAME = "AtlasCrazyflie"

# CONSTANTS
YAW_THRESHOLD = 0.5
VELOCITY_THRESHOLD = 0.75
POSITION_THRESHOLD = 3.14
WEIGHTING = 0.5
beam_length = 0.4
ANGLE_GAIN = 0.125

uris = [
    'radio://0/81/2M/E6E7E6E7E6', #P-Body ON THE RIGHT
    'radio://0/80/2M/E7E7E7E7E7', #Atlas ON THE LEFT
]

reference = np.array([0, 0, 0])  # reference state
height = 0.8

#CONTROL QUEUE DEFINITIONS
Takeoff = namedtuple('Takeoff', ['height', 'time'])
Land = namedtuple('Land', ['time'])
Altitude = namedtuple('Goto', ['roll', 'pitch', 'yaw', 'altitude'])

# Reserved for the control loop, do not use in sequence
Quit = namedtuple('Quit', [])

# CONTROLLER SPECIFIC
# Kr = matrix = np.array([
#     [6.32455532033701, -1.92611708090559e-14, -2.51364697870046e-15],
#     [-5.11540256539333e-14, 5.99999999999965, 1.87222361954461e-14],
#     [-4.75035904976474e-15, -1.2170348703185e-12, 1.99999999999997]
# ])

# Kx = np.array([
#     [26.6274133227985, 43.4037667564728, -7.76461230836858e-14, -1.52606537403102e-13, -3.10094451126148e-14, -7.2568445419908e-14],
#     [1.9122942608089e-14, 7.80944765291056e-14, 26.9778082342948, 45.6501707938696, 9.5871237349572e-14, 1.54384453940806e-13],
#     [-2.2128302669098e-13, -3.22789743035866e-13, -5.46392521542712e-12, -9.16730358643293e-12, 13.563245018237, 20.9904016551837]
# ])

# test 004
# Kr = matrix = np.array([
#     [0.774596669241478, -7.57869734094658e-15, 2.48569466688542e-15],
#     [1.43229129293392e-16, 0.774596669241466, 1.74517030662685e-15],
#     [-1.82962367597883e-14, 3.50430342890093e-14, 1.4142135623731]
# ])

# Kx = np.array([
#     [14.8203309199592, 38.4988727916904, -1.17193125186401e-13, -2.48986395355086e-13, 1.98705962319181e-14, 2.73892732865911e-14],
#     [1.07662524994272e-14, 2.78180472623526e-14, 16.7226731259642, 38.5024797661155, 1.75468728301679e-14, 2.21988550915834e-14],
#     [-2.75293494341881e-13, -6.07006840732273e-13, 1.02641173125862e-12, 2.55456252800242e-12, 12.3978342486397, 18.98804015896]
# ])

#pretty damn good - test 005
# Kr = matrix = np.array([
#     [0.774596669241478, -7.57869734094658e-15, 2.48569466688542e-15],
#     [1.43229129293392e-16, 0.774596669241466, 1.74517030662685e-15],
#     [-1.82962367597883e-14, 3.50430342890093e-14, 1.4142135623731]
# ])

# Kx = np.array([
#     [14.8203309199592, 38.4988727916904, -1.17193125186401e-13, -2.48986395355086e-13, 1.98705962319181e-14, 2.73892732865911e-14],
#     [1.07662524994272e-14, 2.78180472623526e-14, 16.7226731259642, 38.5024797661155, 1.75468728301679e-14, 2.21988550915834e-14],
#     [-2.75293494341881e-13, -6.07006840732273e-13, 1.02641173125862e-12, 2.55456252800242e-12, 12.3978342486397, 18.98804015896]
# ])

# test 006 - higher reference tracking and lower cost of x,y usage (R[0] R[1] reduced)
# Kr = matrix = np.array([
#     [4.47213595500079, -1.8967683320921e-14, -3.50456964462182e-14],
#     [2.6293003263237e-13, 4.47213595499975, 9.06520416523829e-13],
#     [1.04110497249628e-13, 2.36015136098926e-13, 2.4494897427831]
# ])

# Kx = np.array([
#     [51.8497433567121, 121.686352093861, 1.81748043063742e-13, 5.53418333850863e-13, -1.41356425288108e-13, -1.92214048719555e-13],
#     [3.05610238083765e-12, 7.05058795093076e-12, 57.3447937525281, 121.689648545881, 5.14071864671325e-12, 7.02550213697767e-12],
#     [1.15947326189431e-12, 2.64395874483217e-12, 2.90176902216505e-12, 6.24696614581105e-12, 13.8935425005884, 18.9897738308771]
# ])

# test 007 - higher reference tracking
# Kr = matrix = np.array([
#     [8.94427190999975, 2.26027647643416e-15, -3.47098352104662e-14],
#     [-4.98640417367753e-14, 8.94427190999917, 6.07601034151392e-14],
#     [-4.85582642979611e-14, 8.1663522189814e-16, 4.89897948556577]
# ])

# Kx = np.array([
#     [61.4564468802762, 121.692115046713, 1.32168392247924e-14, -2.66292340789414e-13, -1.43416842003026e-14, 2.01198164929981e-14],
#     [-3.08969439855544e-13, -4.80254485892754e-13, 66.1584863761991, 121.694935647565, 1.85108907074061e-13, 2.19773409056855e-13],
#     [-4.12151782920404e-13, -1.02562663009264e-12, -1.73817938505381e-13, -1.77908826190914e-13, 16.9143533255377, 18.9932747630551]
# ])

# test 008 - increased cost of x,y
# Kr = matrix = np.array([
#     [2.82842712474624, -2.04896689678215e-14, 1.63328049092413e-15],
#     [9.76011587503022e-15, 2.82842712474611, -9.54078316277712e-14],
#     [1.91544271492112e-13, -1.02198028309885e-14, 4.89897948556641]
# ])

# Kx = np.array([
#     [19.4379031086129, 38.507627447592, -2.38025412506778e-13, -4.75242733411628e-13, 3.41048569666809e-14, 3.70870318088626e-14],
#     [7.46041095315908e-14, 1.44900775592534e-13, 20.9248183299666, 38.5104461311937, -3.20107544207595e-13, -3.70009236529581e-13],
#     [1.29058886839783e-12, 2.56893560557008e-12, -2.4706863544209e-13, -4.73003247625247e-13, 16.91435332554, 18.9932747630578]
# ])

# test 009 - half way between on cost of x,y - pretty good reference tracking
# Kr = matrix = np.array([
#     [3.99999999999997, 7.99988795908528e-14, -3.84704897448061e-14],
#     [5.19878888316586e-14, 4.00000000000002, 8.23149850131858e-14],
#     [-1.05538682262459e-13, -1.3848298162725e-13, 4.89897948556622]
# ])

# Kx = np.array([
#     [27.4871243822134, 54.44274355041, 6.04569676324578e-13, 1.07721628423106e-12, -1.39323384783852e-13, -1.5831753177288e-13],
#     [3.61992034880406e-13, 7.40182091502545e-13, 29.5899402304137, 54.4455630549309, 2.80316488354124e-13, 3.13622010729586e-13],
#     [-7.23280157839693e-13, -1.43498041657934e-12, -9.21483877042996e-13, -1.51584951189912e-12, 16.9143533255391, 18.9932747630568]
# ])

#test 010 - slightly reduced actuator usage cost for just y
# Kr = matrix = np.array([
#     [4.00000000000002, -9.37823689312343e-14, -4.28023383463016e-14],
#     [-1.82512387677256e-15, 5.65685424949229, -5.04332386641424e-15],
#     [2.13080076666603e-13, -5.11161026000935e-13, 4.89897948556638]
# ])

# Kx = np.array([
#     [27.4871243822138, 54.4427435504107, -7.0288399991998e-13, -1.36029761836762e-12, -1.49293825662015e-13, -1.60766873241981e-13],
#     [8.87397297289851e-15, -1.43454666898252e-14, 41.8442727114127, 76.9812193972651, -7.57244562571302e-15, -2.55057817358442e-14],
#     [1.39783166824994e-12, 2.71840833889413e-12, -3.89722506184606e-12, -7.17862244144025e-12, 16.9143533255398, 18.9932747630576]
# ])

# test 011 - increased x and y displacement cost - reverted y usage cost
# Kr = matrix = np.array([
#     [3.99999999999988, -1.19226791558174e-14, 3.1183203998837e-14],
#     [3.45396807303626e-14, 3.99999999999997, 2.96771624296977e-14],
#     [-3.7400003186444e-15, 2.42976353451642e-13, 4.89897948556654]
# ])

# Kx = np.array([
#     [31.5529018897375, 54.4481949079311, -1.01024521058057e-13, -1.63396534834487e-13, 1.20285085784515e-13, 1.2677809532453e-13],
#     [2.49757041809788e-13, 4.41331453283056e-13, 34.5777104493298, 54.4522501897105, 1.1370457725448e-13, 1.2771200468766e-13],
#     [-2.4350281408277e-13, -4.98385958454347e-13, 2.09363839551529e-12, 3.30379120353514e-12, 16.9143533255402, 18.9932747630581]
# ])

# test 012 - decreased displacement costs
# Kr = matrix = np.array([
#     [3.99999999999985, 6.02935310104159e-14, -6.60174453900406e-14],
#     [5.63077780588765e-14, 3.99999999999962, -4.60271008239587e-13],
#     [2.29808595158886e-13, -1.73133422015245e-13, 4.89897948556692]
# ])

# Kx = np.array([
#     [24.4030515123744, 54.438608089453, 3.94177831669407e-13, 8.57645638458495e-13, -2.07120944467624e-13, -2.58415028850876e-13],
#     [3.5434061759411e-13, 8.31765911301022e-13, 25.6031599085553, 54.4402173628918, -1.57845601997971e-12, -1.77931048954548e-12],
#     [1.37876795045581e-12, 2.92728522976248e-12, -1.09535226722525e-12, -2.55662868751019e-12, 16.9143533255416, 18.9932747630596]
# ])

# test 013 - further decreased
# Kr = matrix = np.array([
#     [4.00000000000046, -7.27402175993604e-14, 8.52298055843592e-14],
#     [-1.49873134305044e-13, 3.99999999999964, 5.64247654580615e-13],
#     [5.64362005052545e-14, 1.37781236638075e-13, 4.89897948556675]
# ])

# Kx = np.array([
#     [22.7044202824837, 54.4363302454483, -4.47477161565069e-13, -1.10610618265388e-12, 3.4657709538075e-13, 3.86495063972241e-13],
#     [-7.91155020037204e-13, -1.9473294788726e-12, 23.1407731415273, 54.4369153984605, 1.94075995528811e-12, 2.18291001636605e-12],
#     [4.0690084981235e-13, 9.78497739607554e-13, 9.846302565018e-13, 2.14861890066504e-12, 16.9143533255411, 18.9932747630591]
# ])

# test 014 - increased y, x stays the same
# Kr = matrix = np.array([
#     [4.0000000000001, -5.39653758677295e-14, -9.85966142286092e-14],
#     [-2.10800983995702e-13, 4.00000000000036, -2.76060900848238e-14],
#     [7.99040592767474e-14, -7.44727407316869e-14, 4.89897948556733]
# ])

# Kx = np.array([
#     [22.7044202824817, 54.4363302454433, -3.2115903095384e-13, -8.03883463049089e-13, -3.59968265963512e-13, -3.88421462339187e-13],
#     [-1.24460363370555e-12, -3.00566106037366e-12, 24.2385469523241, 54.4383874949943, -9.16320726176051e-14, -1.10805455686927e-13],
#     [4.82737705126062e-13, 9.6893571132747e-13, -4.41619619539758e-13, -8.84968042836494e-13, 16.9143533255427, 18.993274763061]
# ])

# test 015 - decreased velocity penalty for x and y
# Kr = matrix = np.array([
#     [3.99999999999993, -1.94280326626903e-14, 3.2353271450506e-14],
#     [8.32205746840415e-14, 3.9999999999997, -9.54317225448862e-14],
#     [-2.05059874141872e-13, 9.31486489023323e-14, 4.89897948556607]
# ])

# Kx = np.array([
#     [21.2620195896928, 46.5091773290613, -1.55054178154426e-13, -4.1233771846746e-13, 1.08296431255871e-13, 1.23618907777439e-13],
#     [4.9883839061279e-13, 1.09016292437539e-12, 22.8930984659351, 46.5117373713812, -3.41464414501216e-13, -3.87540890508098e-13],
#     [-1.02520415311095e-12, -1.99946277253419e-12, 6.45993620480141e-13, 1.30559598394398e-12, 16.9143533255388, 18.9932747630563]
# ])

# test 016/017 - increased vel penalty
# Kr = matrix = np.array([
#     [3.9999999999999, -7.61528139580911e-14, -1.28235783571842e-14],
#     [6.81669873680934e-15, 3.9999999999995, 1.32396995650341e-13],
#     [1.78627361695528e-13, 9.46333241530934e-14, 4.89897948556625]
# ])

# Kx = np.array([
#     [23.8909647251019, 61.3472671370092, -4.95936567842637e-13, -1.15149560488644e-12, -2.82751425300792e-14, -3.87372891163613e-14],
#     [7.09771864116328e-14, 1.39378426001042e-13, 25.3533452731641, 61.3490072675461, 4.83227382096975e-13, 5.26398904603496e-13],
#     [1.09654035666231e-12, 2.82184914257781e-12, 8.78798374141004e-13, 2.26223979163312e-12, 16.9143533255394, 18.9932747630571]
# ])

# test 018 - lower state reg for y (to match x)
# Kr = matrix = np.array([
#     [3.99999999999981, -2.11919527948859e-13, -5.42859603503687e-14],
#     [3.71023556812709e-13, 4.47213595499923, -5.60761862803788e-14],
#     [8.80524315766114e-14, -2.67396632273618e-13, 4.89897948556646]
# ])

# Kx = np.array([
#     [23.8909647251012, 61.3472671370076, -1.35543083159823e-12, -3.38442842650895e-12, -1.51395656462791e-13, -1.77924797772822e-13],
#     [2.16196032649668e-12, 5.44315392144144e-12, 25.0742756143059, 61.3486751979896, -1.60209330299259e-13, -1.9205208798822e-13],
#     [4.29659738984316e-13, 1.16677417699627e-12, -1.03015547427795e-12, -2.62873740653908e-12, 16.91435332554, 18.9932747630579]
# ])

# test 019 - increased y ref tracking penalty - reverted y state reg
# Kr = matrix = np.array([
#     [4.00000000000001, -7.98954163437356e-14, 1.78478225687334e-14],
#     [-7.13954906895006e-14, 5.65685424949215, -4.1798744286052e-14],
#     [9.92671826390434e-14, -6.58410202444401e-14, 4.89897948556638]
# ])

# Kx = np.array([
#     [23.8909647251021, 61.3472671370092, -4.00671222918332e-13, -8.10266695953042e-13, 1.19026042184689e-13, 1.27530465191973e-13],
#     [-4.93674420926274e-13, -1.30800581281693e-12, 29.0884022983171, 61.3534515000979, -1.49262108864675e-13, -1.60834549114634e-13],
#     [4.42916563886352e-13, 1.16249559570783e-12, -9.22746674755049e-13, -1.97249669236049e-12, 16.9143533255397, 18.9932747630575]
# ])

# test 020 - decreased y ref
# Kr = matrix = np.array([
#     [3.9999999999999, -1.98606646600778e-13, 3.9409818597605e-14],
#     [-1.18654507779688e-13, 2.82842712474641, 7.18557947711188e-14],
#     [3.48529293570742e-13, -1.60674712717841e-13, 4.8989794855664]
# ])

# Kx = np.array([
#     [23.8909647251019, 61.3472671370087, -1.58918846039702e-12, -4.27960857184574e-12, 1.08199244845635e-13, 1.29712293808299e-13],
#     [-8.24460507779486e-13, -2.29232250459784e-12, 22.3388034994352, 61.3454201197482, 2.56181031163677e-13, 2.84958547057541e-13],
#     [2.03403897591785e-12, 5.31980769147419e-12, -1.11732775138081e-12, -3.27218639589499e-12, 16.9143533255397, 18.9932747630575]
# ])

# test 021 - increased again, but by a lesser amount
# Kr = matrix = np.array([
#     [3.99999999999996, 8.22689011055884e-14, -1.31681393758206e-14],
#     [6.18669017319917e-14, 4.898979485566, -1.33425437678956e-14],
#     [-1.06288026478817e-13, 1.11143395892224e-13, 4.89897948556651]
# ])

# Kx = np.array([
#     [23.8909647251022, 61.3472671370104, 4.40276343550299e-13, 1.03926985019209e-12, -1.88040733957853e-14, -4.96096533643612e-14],
#     [3.48190318531619e-13, 9.46451004573669e-13, 27.4430226127195, 61.3514937517344, -7.77783213773131e-14, -7.594892891062e-14],
#     [-5.59095850588838e-13, -1.31762612510689e-12, 1.58609224981107e-12, 4.07235070758917e-12, 16.9143533255402, 18.9932747630581]
# ])

# test 022 - increase x, slightly decrease y ref
# Kr = matrix = np.array([
#     [4.47213595499939, 9.63160266981603e-14, -5.25354831853068e-14],
#     [-5.43087529523332e-14, 4.47213595499971, 1.08352977199173e-13],
#     [-5.3933289073914e-14, -2.34878292789641e-13, 4.89897948556653]
# ])

# Kx = np.array([
#     [25.0742756143069, 61.3486751979927, 4.84089737294442e-13, 9.27313255369689e-13, -1.21074783377803e-13, -1.17554654926483e-13],
#     [-2.92771721082536e-13, -6.4653822749669e-13, 26.4713839221707, 61.3503376220369, 3.70289292897101e-13, 4.18916462393144e-13],
#     [-3.11140912263073e-13, -1.05352967833984e-12, -1.48443449944132e-12, -3.24301741102734e-12, 16.9143533255403, 18.9932747630582]
# ])

# test 023 - slight increase for x actuator cost
# Kr = matrix = np.array([
#     [3.77964473009229, -3.59676486439074e-14, 5.27467375416366e-14],
#     [-3.43321353875441e-14, 4.47213595500027, 4.86237496828447e-15],
#     [2.40889215207143e-14, 1.20542066680277e-12, 4.89897948556664]
# ])

# Kx = np.array([
#     [21.1924548827565, 51.8537137248912, -1.07070313190412e-13, 7.84110055506444e-14, 1.69770185767077e-13, 1.84730126103615e-13],
#     [-2.31388991150494e-13, -4.49393559099648e-13, 26.4713839221741, 61.3503376220447, 3.38926721161976e-14, 3.50235070371734e-14],
#     [9.57015269872896e-13, 2.65172087848138e-12, 7.1733593363333e-12, 1.67504657906806e-11, 16.9143533255406, 18.9932747630586]
# ])

# test 024 - decrease y actuator cost
# Kr = matrix = np.array([
#     [3.77964473009223, 1.25036684841228e-13, -1.19589078239442e-14],
#     [4.09432651451092e-14, 5.77350269189628, 2.66104329642578e-13],
#     [1.15175203664987e-13, -2.029934285098e-13, 4.89897948556763]
# ])

# Kx = np.array([
#     [21.1924548827561, 51.8537137248903, 1.85234809803285e-13, 8.12325030260722e-13, -5.64039946903954e-14, -8.60259213205769e-14],
#     [2.99657950551208e-13, 9.41001951033342e-13, 34.1728608481362, 79.193779731248, 1.09672112851772e-12, 1.17129920613858e-12],
#     [4.74038725077172e-13, 1.31565617711435e-12, -3.58758710569718e-13, -2.32587184101815e-13, 16.9143533255436, 18.9932747630622]
# ])

# test 025 - decrease y act cost, increased x act cost
# Kr = matrix = np.array([
#     [3.53553390593281, -1.23996330081772e-14, -8.46583556844566e-14],
#     [1.94193507232579e-14, 10.0000000000001, 1.90124603177526e-13],
#     [-1.50555413584043e-13, -2.24850284804021e-12, 4.89897948556586]
# ])

# Kx = np.array([
#     [19.8240700822207, 48.5066346599646, -1.20786359318442e-14, 2.93978507392177e-13, -1.72858956320708e-13, -1.46658947957239e-13],
#     [-1.48578458331325e-14, -3.41989458470334e-13, 59.1852344819453, 137.144591734139, 6.55035705212589e-13, 7.16018983168648e-13],
#     [-7.51757332560846e-13, -2.5241676982334e-12, -1.26051296994236e-11, -2.86044823938811e-11, 16.9143533255388, 18.9932747630563]
# ])

# test 026 - decreased y act cost, increased y ref tracking
Kr = matrix = np.array([
    [3.53553390593262, 3.89250244743641e-14, 2.33736911779375e-14],
    [3.35463021966156e-14, 16.733200530681, -1.07275528875179e-13],
    [7.02983000276608e-14, 1.72706085402703e-13, 4.89897948556475]
])

Kx = np.array([
    [19.8240700822196, 48.5066346599617, 1.82072093161377e-13, 5.56286585882067e-13, 8.76428189896241e-14, 1.04964914817347e-13],
    [7.6915627754203e-14, -1.26536432822293e-14, 89.5014160126164, 193.940878556323, -3.74809861976736e-13, -4.31978685153102e-13],
    [2.25756112139253e-13, 6.46380800482222e-13, 9.52579308322653e-13, 2.15418032736892e-12, 16.9143533255342, 18.9932747630512]
])

# eg. figure_eight(2, 1, 45 ...) = 2m width, 1m height, 45s total time
def figure_eight(width, height, total_time, time, time_offset = 0, horizontal = True):
    if horizontal:
        width_freq_modifier = 1
        height_freq_modifier = 2
    else:
        width_freq_modifier = 2
        height_freq_modifier = 1
    reference = np.array([0.5 * width * np.sin((2 * np.pi * width_freq_modifier/total_time) * (time - time_offset)), - 0.5 * height * np.sin((2 * np.pi * height_freq_modifier/total_time) * (time - time_offset)), 0]) 
    return reference

def step_offset(elapsed_time, step_time, start_x=0, start_y=0, start_yaw=0, x=0, y=0,yaw=0):
    if elapsed_time > step_time:
        reference = np.array([x,y,yaw])
    else:
        reference = np.array([start_x,start_y,start_yaw])
    return reference

def control_thread():
    vicon = vi()
    vicon_thread = threading.Thread(target=vicon.main_loop)
    vicon_thread.start()
    print("starting vicon thread")

    logger = logging.getLogger('crazyflie_yaw_test')
    logger.setLevel(logging.INFO)

    # Set up a rotating file handler
    handler = RotatingFileHandler(
        'app.log',  # Log file name
        maxBytes=100000,  # Maximum size of a log file in bytes before rotation
        backupCount=3  # Number of backup files to keep
    )

    # Optional: Set a formatter for the log messages
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)

    # Add the handler to the logger
    logger.addHandler(handler)

    # Write an informational log message
    logging.info('Log started')

    # creates the udp client for plotting
    client = UDP_Client()

    roll = 0
    pitch = 0

    current_pos = np.array([0,0,0,0,0,0])
    prev_pos = np.array([0,0,0,0,0,0])

    pygame.init()
    pygame.joystick.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()
    
    try:
        time.sleep(0.1)
        offset = get_pos(vicon.getPos(OBJECT_NAME), current_pos)
        pbody_offset = get_pos(vicon.getPos("PbodyCrazyFlie"), current_pos)
        atlas_offset = get_pos(vicon.getPos("AtlasCrazyflie"), current_pos)

        print("Offset Pos & Rot: ",offset)
        
        waiting = True
        while(waiting):
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0: # A button
                        waiting = False
            time.sleep(0.25)
            print('waiting')
        
        print("Starting flight")
        logging.info('Starting flight')
            
        
        # takeoff 3 seconds
        controlQueues[0].put(Takeoff(height, 3))
        controlQueues[1].put(Takeoff(height, 3))
        
        time.sleep(3)
        
        flying = True
        start_time = time.time()
        current_pos = get_pos(vicon.getPos(OBJECT_NAME), current_pos) - offset
        prev_pos = current_pos
        current_time = time.time()
        prev_time = current_time
        y_tracker = 0
        prev_vel = np.array([0,0,0,0,0,0])
        print('starting altitude flight')

        while flying:
                
            time.sleep(0.01) #

            # safety timeout
            elapsed_time = time.time() - start_time

            reference = step_offset(elapsed_time, 30, yaw=0)
            

            # time_offset = 10
            # if elapsed_time < time_offset: # time to stabilize
            #     reference = np.array([0, 0, 0])
            # else:
            #     reference = figure_eight(3, 2, 100, elapsed_time, time_offset) # eg. figure_eight(2, 1, 45 ...) = 2m width, 1m height, 45s total time

            # if elapsed_time > 120:
            #     print("Timeout reached. Exiting loop.")
            #     break
            
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 1:  # B button
                        flying = False
                        logging.info('Flight disabled (B Button)')
                    if event.button == 3:  # Y button
                        logging.info('Emergency Stop (Y Button)')
                        raise Exception('Manual Emegency Stop') # emergency stop
            
            d_time = time.time() - current_time
            # update states
            current_pos = get_pos(vicon.getPos(OBJECT_NAME), current_pos) - offset
            
            current_vel = vicon.getVel(OBJECT_NAME)
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            x = np.array([current_pos[0],current_vel[0], current_pos[1],current_vel[1], current_pos[5],current_vel[5]])
            Cx = np.array([current_pos[0], current_pos[1], current_pos[5]])
            
            
            y_tracker += (Kr @ (reference - Cx))*d_time
            # y_tracker = np.clip(y_tracker, -30, 30) ------ causes maximum distance for reference tracking
            #  Try without reference tracking first!
            stateReg = -Kx @ x
            u = stateReg + y_tracker
            
            yaw = current_pos[5]

            ## split the sum of forces into the the agents compomonents
            HT_thrusts = np.array([u[0], u[1]])
            moment_z = u[2]/(2*beam_length)
            
            rot_matrix = np.array([[np.cos(yaw), np.sin(yaw)],
                [-np.sin(yaw), np.cos(yaw)]])
            
            HT_thrusts = rot_matrix @ HT_thrusts
            bx_thrust = HT_thrusts[0]
            by_thrust = HT_thrusts[1]
            bx_1 = WEIGHTING * bx_thrust
            bx_2 = (1-WEIGHTING) * bx_thrust
            by_1 = 0.5 * by_thrust
            by_2 = 0.5 * by_thrust 
            
            roll_1 = 10*ANGLE_GAIN*bx_1
            roll_2 = 10*ANGLE_GAIN*bx_2
            
            pitch_1 = ANGLE_GAIN*(-by_1 - moment_z) # right one
            pitch_2 = ANGLE_GAIN*(-by_2 + moment_z) # left one
            
            # send to drones
            # Breaking down the control queue put operation into smaller parts
            yaw = math.degrees(current_pos[5])

            c_roll_1 = max(min(roll_1, 5), -5)
            c_roll_2 = max(min(roll_2, 5), -5)
            
            c_pitch_1 = max(min(pitch_1, 5), -5)
            c_pitch_2 = max(min(pitch_2, 5), -5)

            # Putting the Altitude command into the control queue
            controlQueues[0].put(Altitude(c_roll_1, c_pitch_1,  yaw, height))  # ATLAS RIGHT
            controlQueues[1].put(Altitude(c_roll_2, c_pitch_2, yaw, height))  # PBODY LEFT
            
            atlas = vicon.getPos("AtlasCrazyflie") - atlas_offset
            pbody = vicon.getPos("PbodyCrazyFlie") - pbody_offset
            
            atlasUF = vicon.getUF("AtlasCrazyflie") - atlas_offset
            pbodyUF = vicon.getUF("PbodyCrazyFlie") - pbody_offset

            client.send({
                OBJECT_NAME: {
                    "position": {"x": float(current_pos[0]),"y": float(current_pos[1]), "z": float(current_pos[2])},
                    "attitude": {"roll": math.degrees(float(current_pos[3])),"pitch": math.degrees(float(current_pos[4])),"yaw": math.degrees(float(current_pos[5]))},
                    "velocityT": {"x": float(current_vel[0]),"y": float(current_vel[1]), "z": float(current_vel[2])},
                    "velocityR": {"roll": math.degrees(float(current_vel[3])),"pitch": math.degrees(float(current_vel[4])),"yaw": math.degrees(float(current_vel[5]))},
                },
                "CONTROLLER":{
                    "reference": {"x": float(reference[0]),"y": float(reference[1]), "yaw": math.degrees(float(reference[2]))},
                    "wrench": {"x": float(u[0]), "y": float(u[1]), "moment z": float(u[2])},
                    "Rotated Thrusts": {"x" : float(HT_thrusts[0]), "y": float(HT_thrusts[1])},
                    "Queue Size": {"Atlas": controlQueues[0].qsize() , "P-Body":controlQueues[1].qsize() },
                    "controller Components": {"Reference Tracking x": float(y_tracker[0]),"Reference Tracking y": float(y_tracker[1]),"Reference Tracking z": float(y_tracker[2]), "State Regulation x": float(stateReg[0]),"State Regulation y": float(stateReg[1]),"State Regulation z": float(stateReg[2])},  
                },
                "ATLAS":{
                    "setpoint": {"clamped roll": float(c_roll_1),"clamped pitch": float(c_pitch_1),"yaw": float(yaw)},
                    "position": {"x": float(atlas[0]),"y": float(atlas[1]), "z": float(atlas[2])},
                    "attitude": {"roll": math.degrees(float(atlas[3])),"pitch": math.degrees(float(atlas[4])),"yaw": math.degrees(float(atlas[5]))},
                    "unfiltered": {"x": float(atlasUF[0]),"y": float(atlasUF[1]), "z": float(atlasUF[2]), "roll": math.degrees(float(atlasUF[3])),"pitch": math.degrees(float(atlasUF[4])),"yaw": math.degrees(float(atlasUF[5]))}
                },
                "P-BODY":{
                    "setpoint": {"clamped roll": float(c_roll_2),"clamped pitch": float(c_pitch_2),"yaw": float(yaw)},
                    "position": {"x": float(pbody[0]),"y": float(pbody[1]), "z": float(pbody[2])},
                    "attitude": {"roll": math.degrees(float(pbody[3])),"pitch": math.degrees(float(pbody[4])),"yaw": math.degrees(float(pbody[5]))},
                    "unfiltered": {"x": float(pbodyUF[0]),"y": float(pbodyUF[1]), "z": float(pbodyUF[2]), "roll": math.degrees(float(pbodyUF[3])),"pitch": math.degrees(float(pbodyUF[4])),"yaw": math.degrees(float(pbodyUF[5]))}
                },
            })
        
        controlQueues[0].put(Land(3))
        controlQueues[1].put(Land(3))

        # controlQueues[1].put(Land(3))
    except Exception as e:
        for ctrl in controlQueues:
            ctrl.put(Quit())
        print(traceback.format_exc())
        logging.exception(traceback.format_exc())
        print(e)
    finally:
        vicon.end()
        client.close()

def update_crazy_controller(scf):
    pr = cProfile.Profile()
    pr.enable()
    
    control = controlQueues[uris.index(scf.cf.link_uri)]
    while True:
        command = control.get()
        if type(command) is Quit:
            scf.cf.high_level_commander.stop()
            time.sleep(0.1)
            pr.disable()
            stats = pstats.Stats(pr)
            stats.sort_stats(pstats.SortKey.CUMULATIVE)
            pr.dump_stats('output.prof')
            stream = open('output.txt', 'w')
            stats = pstats.Stats('output.prof', stream=stream)
            stats.sort_stats(pstats.SortKey.CUMULATIVE)
            stats.print_stats()
            return
        elif type(command) is Takeoff:
            scf.cf.high_level_commander.takeoff(command.height, command.time)
        elif type(command) is Land:
            scf.cf.high_level_commander.land(0.0, command.time)
            time.sleep(0.1)
        elif type(command) is Altitude:

            scf.cf.commander.send_custom_altitude_setpoint(command.roll, command.pitch ,command.yaw, command.altitude)
        else:
            scf.cf.high_level_commander.stop()
            time.sleep(0.1)

            print('Warning! unknown command {} for uri {}'.format(command,cf.uri))

def light_check(scf):
    def activate_led_bit_mask(scf):
        scf.cf.param.set_value('led.bitmask', 255)

    def deactivate_led_bit_mask(scf):
        scf.cf.param.set_value('led.bitmask', 0)
    
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(2)

def take_off(scf):
    commander= scf.cf.high_level_commander
    commander.takeoff(height, 2.0)
    time.sleep(3)
    
def land(scf):
    scf.cf.commander.send_notify_setpoint_stop()
    scf.cf.high_level_commander.land(0.0, 5.0)
    time.sleep(5)

    scf.cf.high_level_commander.stop()
    time.sleep(0.1)

def param_stab_est_callback(name, value):
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)

def set_params(scf):
    groupstr = "flightmode"
    namestr = "stabModeYaw"
    full_name = groupstr + '.' + namestr
    # scf.param.add_update_callback(group=groupstr, name=namestr,
    #                              cb=param_stab_est_callback)
    scf.cf.param.set_value(full_name, 1)

    namestr = "althold"
    full_name = groupstr + '.' + namestr
    # scf.param.add_update_callback(group=groupstr, name=namestr,
    #                              cb=param_stab_est_callback)
    scf.cf.param.set_value(full_name, 1)
    
# EMERGENCY STOP
def stop(scf):
    commander= scf.cf.high_level_commander
    commander.stop()
    time.sleep(0.1)

    print('EMERGENCY STOP OCCURED')
    
def hold_pos(scf):
    commander= scf.cf.high_level_commander
    commander.go_to(0, 0, 0, 0, 1.0, relative=True)
    time.sleep(2)

def get_pos(position, current_pos):
    if position is not None:
        return np.array(position)
    else:
        return current_pos
        print('LOST SIGHT OF OBJECT')

if __name__ == '__main__':
    # controlQueues = [TimedQueue(0.1) for _ in range(len(uris))]
    controlQueues = [Queue() for _ in range(len(uris))]

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    
    with Swarm(uris, factory=factory) as swarm:
        try:    
            swarm.parallel_safe(set_params)
            swarm.parallel_safe(light_check)
            print('Light check done')
            swarm.reset_estimators()
            print('Estimators reset')
            
            threading.Thread(target=control_thread).start()
            print('Control thread started')
            swarm.parallel_safe(update_crazy_controller)
            print('Crazyflie threads started')
        except Exception as e:
            swarm.parallel_safe(stop)
            time.sleep(0.1)

            print(traceback.format_exc())
        
    pygame.quit()
    # Convert results to numpy arrays for easier handling
        
