# upyduino (Micro-pyduino) is a pyduino port for micropython on pyboard
# by X. HINAULT - www.mon-club-elec.fr - 2015 -2017 

# Micro-python, python for microcontrollers, is written by D. Georges
# http://docs.micropython.org/en/latest/pyboard/quickref.html

"""
 * Copyright (c) 2015-2017 by Xavier HINAULT - support@mon-club-elec.fr
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 3
 * or the GNU Lesser General Public License version 3, both as
 * published by the Free Software Foundation.
 
"""

import pyb

from math import sqrt, sin, cos, tan, radians, degrees, pi

LOW=0
HIGH=1

OUTPUT=0
INPUT=1
INPUT_PULLUP=2

A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11 =0,1,2,3,4,5,6,7,8,9,10,11 # identifiants broches analogiques

PWM0, PWM1, PWM2, PWM3, PWM4, PWM5 =0,1,2,3,4,5 # identifiants broches PWM

pins=[pyb.Pin.board.X1,pyb.Pin.board.X2, pyb.Pin.board.X3, pyb.Pin.board.X4, pyb.Pin.board.X5, pyb.Pin.board.X6, pyb.Pin.board.X7, pyb.Pin.board.X8, pyb.Pin.board.Y9, pyb.Pin.board.Y10, pyb.Pin.board.Y11, pyb.Pin.board.Y12, pyb.Pin.board.Y1, pyb.Pin.board.Y2, pyb.Pin.board.Y3, pyb.Pin.board.Y5, pyb.Pin.board.Y6, pyb.Pin.board.Y7, pyb.Pin.board.Y8, pyb.Pin.board.X9, pyb.Pin.board.X10, pyb.Pin.board.X11, pyb.Pin.board.X12, pyb.Pin.board.X17, pyb.Pin.board.X18, pyb.Pin.board.X19, pyb.Pin.board.X20, pyb.Pin.board.X21, pyb.Pin.board.X22, pyb.Pin.cpu.B4, pyb.Pin.cpu.A15, pyb.Pin.cpu.A14, pyb.Pin.cpu.A13] # pins are named with pins[i] where indice is idem Arduino
# 0 à 11 côté droit (X1 - Y12) puis 12 à 23 côté gauche (Y1 à X12) puis 24-29 côté bas (X17 à X22) + LEDs onboard P2-P3-P4-P5 en dernier

LED4, LED3, LED2, LED1 = len(pins)-4, len(pins)-3, len(pins)-2, len(pins)-1 # identifiant des Leds ON BOARD - ordre idem micropython

analogPins=[pyb.Pin.board.X1,pyb.Pin.board.X2, pyb.Pin.board.X3, pyb.Pin.board.X4, pyb.Pin.board.X5, pyb.Pin.board.X6, pyb.Pin.board.X7, pyb.Pin.board.X8, pyb.Pin.board.Y11, pyb.Pin.board.Y12, pyb.Pin.board.X11, pyb.Pin.board.X12]

adcList=[None, None, None, None, None, None, None, None, None,None, None, None] # list pour objets adc - jusqu'à 1 objet adc par broche analogique

pwmPins=[[pyb.Pin.board.X1,5,1],[pyb.Pin.board.X2,5,2],[pyb.Pin.board.X3,5,3],[pyb.Pin.board.X4,5,4], [pyb.Pin.board.X6,2,1], [pyb.Pin.board.X7,13,1], [pyb.Pin.board.X8,8,1]] # [pin,timer, channel]

pwmchannelList=[None, None, None, None, None] # list channel pour PWM

tonePinsList=[]
toneTimersList=[]

# GPIO functions
#http://docs.micropython.org/en/latest/library/pyb.Pin.html#pyb-pin

def pinMode(pinIn, modeIn): # pinIn : pin index in pins list, modeIn : mode between OUTPUT, INPUT, INPUT_PULLUP
    if modeIn==OUTPUT : pins[pinIn].init(pyb.Pin.OUT_PP) # set pin as output
    elif modeIn==INPUT : pins[pinIn].init(pyb.Pin.IN) # set pin as input
    elif modeIn==INPUT_PULLUP : pins[pinIn].init(pyb.Pin.IN, pull=pyb.Pin.PULL_UP) # set pin as input with pullup

def digitalWrite(pinIn, stateIn):
    if stateIn==LOW:pins[pinIn].low()
    elif stateIn==HIGH:pins[pinIn].high()

def digitalRead(pinIn):
    return pins[pinIn].value()
    
def toggle(pinIn): # inverse l'état de la broche 
    if pins[pinIn].value() : pins[pinIn].low()
    else: pins[pinIn].high()

def tone(pinIn, freq, timer=1): # applique fréquence carrée sur broche
    
    global tonePinsList, toneTimersList
    
    # ajout à la list des broches Tone si pas déjà utilisée
    if pinIn not in tonePinsList:
        tonePinsList.append(pinIn)
    
    #print (tonePinsList) # debug
    
    # création du timer Tone si pas déjà utilisé
    if timer not in toneTimersList : 
        toneTimersList.append(timer)
        toneTimer=pyb.Timer(timer, freq=2*freq) # timer 2 fois plus rapide que freq pour 50% haut / 50% bas
        toneTimer.callback(lambda t: toggle(pinIn))
    else:
        print("Timer indisponible")

    #print (toneTimersList) # debug

def noTone(pinIn):
    
    global tonePinsList, toneTimersList
    
    # retire la broche de la list des broches Tone
    if pinIn in tonePinsList:
        idx=tonePinsList.index(pinIn) # récupère l'index dans la list
        #print(idx) # debug
        tonePinsList.remove(pinIn) # retire la broche (elle-même pas l'index)
        pyb.Timer(toneTimersList[idx]).deinit() # désactive le timer AVANT de l'enlever de la list
        toneTimersList.remove(toneTimersList[idx]) # timer a le même indice que pin à priori
        
    #print (tonePinsList) # debug
    #print (toneTimersList) # debug

# ANALOG functions
# 12 analogs pins with 12 bits resolution in 3.3V (resolution=0.8mV) !

def analogRead(analogPinIn): # recoit index entre 0 et 11 sous la forme A0-A11
    """
    if pins[pinIn] not in analogPins :
        print ("This pin is not analog. analog pins are : ", str(analogPins))
    else:
        
        if pins[pinIn].mode()!=pyb.Pin.ANALOG: # test pin mode
            print ("Config Analog Pin")
            pins[pinIn].init(pyb.Pin.ANALOG) # set pin as analog
        """
    
    # utiliser objet ADC plutôt : http://docs.micropython.org/en/latest/library/pyb.ADC.html#pyb-adc
    global adcList # utilise la liste des objets adc déclarée initialement

    #print (adcList[analogPinIn]) # debug

    if adcList[analogPinIn]==None : # si l'objet adc n'existe pas pour cette broche
        adcList[analogPinIn]=pyb.ADC(analogPins[analogPinIn]) # crée l'objet adc associé à la broche
    
    return adcList[analogPinIn].read() # renvoie la valeur analogique de la broche

def analogWrite(pwmPin, pulsewidthIn, freq=1000): # pulse 0-255

    # voir : http://docs.micropython.org/en/latest/pyboard/quickref.html#pwm-pulse-width-modulation
    
    #print (pwmPins[pwmPin][0]) # debug - la broche
    #print (pwmPins[pwmPin][1]) # debug - le timer
    #print (pwmPins[pwmPin][2]) # debug - le channel
    
    global pwmchannelList # utilise la list des objets channels utilisés

    if pwmchannelList[pwmPin]==None: # si existe pas on crée le channel
        tim = pyb.Timer(pwmPins[pwmPin][1], freq=freq)
        pwmchannelList[pwmPin] = tim.channel(pwmPins[pwmPin][2], pyb.Timer.PWM, pin=pwmPins[pwmPin][0])

    pwmchannelList[pwmPin].pulse_width_percent(pulsewidthIn*100/255)

def analogWritePercent(pwmPin, pulsewidthIn):

    # voir : http://docs.micropython.org/en/latest/pyboard/quickref.html#pwm-pulse-width-modulation
    
    #print (pwmPins[pwmPin][0]) # debug - la broche
    #print (pwmPins[pwmPin][1]) # debug - le timer
    #print (pwmPins[pwmPin][2]) # debug - le channel
    
    global pwmchannelList # utilise la list des objets channels utilisés

    if pwmchannelList[pwmPin]==None: # si existe pas on crée le channel
        tim = pyb.Timer(pwmPins[pwmPin][1], freq=1000)
        pwmchannelList[pwmPin] = tim.channel(pwmPins[pwmPin][2], pyb.Timer.PWM, pin=pwmPins[pwmPin][0])

    pwmchannelList[pwmPin].pulse_width_percent(pulsewidthIn)

## NOTE : Autant que possible, on se base sur les fonctions natives micropython

# Time functions
# delay
def delay(msIn):
    pyb.delay(msIn)

# delayMicroseconds
def delayMicroseconds(usIn):
    pyb.udelay(usIn)

# udelay # variante
def udelay(usIn):
    pyb.udelay(usIn)
    
# millis
def millis():
    return pyb.millis()
    
# micros
def micros():
    return pyb.micros() 
    
# timer ==> renvoie un objet Timer micropython
def timer(nbIn):
    return pyb.Timer(nbIn)

#== RTC == 

# TODO

#== MATH ==

#----------- MATH -------------

#-- min(x,y) --> Python

#-- max(x,y) --> Python

#-- abs(x) --> Python 

#-- constrain(x,a,b)
def constrain(x,valMin,valMax):
    if x < valMin : 
        return valMin

    elif valMax < x :
        return valMax

    else :
        return x

#-- map(valeur, fromLow, fromHigh, toLow, toHigh) --> renommée rescale
def rescale(valeur, in_min, in_max, out_min, out_max):
    return (valeur - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    # d'après la fonction map du fichier wirin.c du core Arduino

#-- pow(x,y) : calcul x à la puissance y --> Python

#-- sq(x) -- calcule le carré de x
def sq(x):
    return pow(x,2)

# fonctions utilisant maths : voir le import
#-- sqrt(x) -- calcule la racine carrée de x --> module math
#def sqrt(x):
    #return math.sqrt(x)
    
#-- sin(x) -- sinus de l'angle en radians --> module math

#-- cos(x) cosinus de l'angle en radians --> module math

#-- tan(x) cosinus de l'angle en radians --> module math

#-- radians(x) --> module math

#-- degrees(x) --> module math

#-- round --> fonction native Python

#-- sum --> fonction native Python 

#-- randomSeed()  initialise le générateur de nombre aléatoire
#def randomSeed(x):
#    rd.seed(x) # appelle fonction seed du module random
    
#-- random(max) et random(min,max) : renvoie valeur aléatoire entière
def random(*arg): # soit forme random(max), soit forme random(min,max)
    # Renvoie une valeur aléatoire entiere
    
    if len(arg)==1:
        #return rd.randint(0,arg[0])
        return round(pyb.rng()*arg[0]/1073741824.0)
        
    elif len(arg)==2: # genere 1 valeur aléatoire entre 2 bornes
        delta=abs(arg[1]-arg[0])
        values=round(pyb.rng()*delta/1073741824.0)
        return arg[0]+values
    else:
         return 0 # si argument invalide

#== gestion de bits et octets == 
def lowByte(a):
    # Renvoie l'octet de poids faible de la valeur a
    out=bin(a) # '0b1011000101100101'
    out=out[2:] # enleve 0b '1011000101100101'
    out=out[-8:] # extrait 8 derniers caracteres - LSB a droite / MSB a gauche 
    while len(out)<8:out="0"+out # complete jusqu'a 8 O/1
    out="0b"+out # re-ajoute 0b 
    return out

def highByte(a):
    # renvoie l'octet de poids fort de la valeur a
    
    
    out=bin(a) # '0b1011000101100101'
    out=out[2:] # enleve 0b '1011000101100101'
    while len(out)>8:out=out[:-8] # tant que plus de 8 chiffres, enleve 8 par 8 = octets low

    # une fois obtenu le highbyte, on complete les 0 jusqu'a 8 chiffres
    while len(out)<8:out="0"+out # complete jusqu'a 8 O/1
    out="0b"+out # re-ajoute 0b 
    return out
    

def bitRead(a, index):
    # lit le bit de rang index de la valeur a
    # le bit de poids faible a l'index 0
    
    out=bin(a) # '0b1011000101100101'
    out=out[2:] # enleve 0b '1011000101100101'
    out=out[len(out)-index-1] # rang le plus faible = indice 0 = le plus a droite
    # extrait le caractere du bit voulu - LSB a droite / MSB a gauche 
    #out="0b"+out # re-ajoute 0b 
    return out
    

def bitWrite(a, index, value):
    # Met le bit d'index voulu de la valeur a a la valeur indiquee (HIGH ou LOW)
    # le bit de poids faible a l'index 0 
    
    out=bin(a) # '0b1011000101100101'
    out=out[2:] # enleve 0b '1011000101100101'
    out=list(out) # bascule en list
    out[len(out)-index-1]=str(value) # rang le plus faible = indice 0 = le plus a droite
    #out=str(out) # rebascule en str - pb car reste format liste
    out="".join(out) # rebascule en str - concatenation des caracteres
    # remplace le caractere du bit voulu - LSB a droite / MSB a gauche 
    out="0b"+out # re-ajoute 0b 
    return out
    

def bitSet(a,index):
    # Met le bit d'index voulu de la valeur a a HIGH
    # le bit de poids faible a l'index 0
    
    return bitWrite(a,index,1) # met le bit voulu a 1 - Index 0 pour 1er bit poids faible
    

def bitClear(a,index):
    # Met le bit d'index voulu de la valeur a a LOW
    # le bit de poids faible a l'index 0 
    
    return bitWrite(a,index,0) # met le bit voulu a 0 - Index 0 pour 1er bit poids faible
    

def bit(index): # calcule la valeur du bit d'index specifie (le bits LSB a l'index 0)
    # calcule la valeur du bit d'index specifie 
    # le bits de poids faible a l'index 0 - calcule en fait 2 exposant index
    
    return pow(2,index) # cette fonction renvoie en fait la valeur 2^index


# préférer print natif micropython... car permet de s'habituer au Python

# forme simpliste
class Serial():
    
    # def __init__(self): # constructeur principal
    
    # note : pas besoin de self dans les classes en upython semble-t-il
    def println(text):  # message avec saut de ligne
        print(text) 

"""
class Serial():
	
	# def __init__(self): # constructeur principal
	
	def println(self,text, *arg):  # message avec saut de ligne
		# Emulation Serial.println dans console systeme
		# Supporte formatage chaine façon Arduino avec DEC, BIN, OCT, HEX
		
		
		# attention : arg est reçu sous la forme d'une liste, meme si 1 seul !
		text=str(text) # au cas où
		
		arg=list(arg) # conversion en list... évite problèmes.. 
		
		#print arg - debug
		
		if not len(arg)==0: # si arg a au moins 1 element (nb : None renvoie True.. car arg existe..)
			if arg[0]==DEC and text.isdigit():
				print(text)
			elif arg[0]==BIN and text.isdigit():
				print(bin(int(text)))
			elif arg[0]==OCT and text.isdigit():
				print(oct(int(text)))
			elif arg[0]==HEX and text.isdigit():
				print(hex(int(text)))
		else: # si pas de formatage de chaine = affiche tel que 
			print(text)
		
		
		# ajouter formatage Hexa, Bin.. cf fonction native bin... 
		# si type est long ou int
	
	#def print(self,text): # affiche message sans saut de ligne
		
		#text=str(txt)
		
		#print(text), # avec virgule pour affichage sans saus de ligne
	
	
	def begin(self,rate): # fonction pour émulation de begin... Ne fait rien... 
		return


# fin classe Serial 
"""
