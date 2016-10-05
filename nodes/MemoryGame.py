#!/usr/bin/env python
#coding: utf-8

from Card import Card
from ActionLog import ActionLog
from GazeLog import GazeLog
from AnimationLog import AnimationLog
from Player import Player
from CardReminded import CardReminded

import random
import rospy
import datetime
import os
from std_msgs.msg import String, UInt8
from memory.msg import Activity, Animation
from os.path import expanduser

# IDLE
# PLAYING

class MemoryGame():

	def __init__(self, nbPlayer = 3, nbCard = 16, firstPlayer = 0, gameNumber = 0):
		
		self.nbPlayer = nbPlayer

		self.nbCard = nbCard
		self.roundGame = 0
		self.gameNumber = gameNumber

		if firstPlayer < 0 or firstPlayer > self.nbPlayer:
			self.player = 0
		else:
			self.player = firstPlayer

		self.gameFinished = False
		self.remainingCard = list()
		self.createCards()

		# create list of action, gaze logs
		self.listActionLog = list()
		self.listGazeLog = list()
		self.listAnimationLog = list()

		# create list of players
		self.listPlayer = list()
		self.createPlayers()

		# create robot memory usefull to implement strategy
		self.listCardReminded = list()

		# create topic receiving informations for the tablet
		TOPIC_LISTENER_TABLET = rospy.get_param('~topic_listener_tablet')
		rospy.Subscriber(TOPIC_LISTENER_TABLET, String, self.callBackChildMove)

		# create topic receiving informations concerning the end of robot animation
		TOPIC_END_ANIMATION = rospy.get_param('~topic_end_animation')
		rospy.Subscriber(TOPIC_END_ANIMATION, Animation, self.callBackAnimationEnded)

		# create topic to receive message gaze match
		TOPIC_GAZE_MATCH = rospy.get_param('~topic_gaze_match')
		rospy.Subscriber(TOPIC_GAZE_MATCH, UInt8, self.callBackGazeMatch)

		# boolean indicating if the robot is doing animation or not
		self.nbCardReturned = 0
		# save the t0 (starting point) of the experiment
		self.t0Experiment = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

	def createPlayers(self):
		idPlayer = 0
		for i in xrange(self.nbPlayer):
			self.listPlayer.append(Player(idPlayer))
			idPlayer += 1

	def getPlayerbyId(self, idPlayer):
		for player in self.listPlayer:
			if player.getId() == idPlayer:
				return player

	def getCardByid(self, idCard):
		for card in self.remainingCard:
			if card.getIndice() == idCard:
				return card

	def createCards(self):

		# get the type of cards to use in function of the game number
		myStr = "m " + str(self.gameNumber)
		publisher_tablet.publish(myStr)
		rospy.sleep(1)
		
		value = 0
		nbValue = self.nbCard/2
		vectorIndice = range(0, self.nbCard)

		for i in xrange(self.nbCard):
			self.remainingCard.append(Card(i, -1))


		while len(vectorIndice) > 0:

			# randomly pick an indice in vectorIndice
			indice = random.choice(vectorIndice)
			# remove this indice from the vector
			vectorIndice.remove(indice)
			# create card with this indice and value
			self.remainingCard[indice].setValue(value)

			# randomly pick an indice in vectorIndice
			indice = random.choice(vectorIndice)
			# remove this indice from the vector
			vectorIndice.remove(indice)
			# create card with this indice and value
			self.remainingCard[indice].setValue(value)

			# increase value by +1
			value += 1

		myStr = "i "
		for card in self.remainingCard:
			myStr += str(card.getValue())
			myStr += " "

		# communicate nb value to position to tablet
		publisher_tablet.publish(myStr)

	def nextPlayer(self):
		self.player += 1

		if self.player >= self.nbPlayer:
			self.player = 0 

		# if child's turn
		if self.player == 0:
			publisher_tablet.publish("y")

	def deleteRemindedCardAtIndex(self, index):

		newList = []
		for card in self.listCardReminded:
			if card.getCard().getIndice() != index:
				newList.append(card)

		self.listCardReminded = newList

	def play(self):
		success = None

		while(self.gameFinished == False):

			if self.player != 0:

				# the robot is playing
				self.nbCardReturned = 0

				# save player of this round
				roundPlayer = self.player

				# get the two cards to be returned
				card1, card2, statePlayed = self.stategyRobot()

				# check if success
				success = self.isPair(card1, card2)

				# launch animation to return first card for player X (which is a robot)
				newMsg = Activity()
				newMsg.player = roundPlayer
				newMsg.state = "playing"
				newMsg.result = int(success)
				newMsg.statePlayed = statePlayed
				publisher_activity.publish(newMsg)

				# wait that the animation for returning 1st card is in progress
				while (self.nbCardReturned == 0):
					rospy.sleep(0.1)

				# at this point, the robot has returned the first card, send this information to the tablet
				myStr = "f " + str(card1.getIndice())
				publisher_tablet.publish(myStr)

				# wait that the animation for returning 2nb card is in progressg
				while (self.nbCardReturned == 1):
					rospy.sleep(0.1)

				# at this point, the robot has returned the second card, send this information to the tablet
				myStr = "s " + str(card2.getIndice()) + " " + str(int(success)) + " " + str(roundPlayer)
				publisher_tablet.publish(myStr)

				# if success = false, important to keep that in mind for later move, if success not needed because robot won't play these cards anymway
				if success == False:
					self.listCardReminded.append(CardReminded(card1, self.roundGame))
					self.listCardReminded.append(CardReminded(card2, self.roundGame))

				# publish that player X is now idle
				newMsg.state = "idle"
				publisher_activity.publish(newMsg)

				# log action
				self.listActionLog.append(ActionLog(card1, card2, roundPlayer, self.roundGame, success, datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
				
				# hack -> to be renoved 
				if self.player == 2:
					rospy.sleep(4)

				# wait a moment
				rospy.sleep(TIME_BETWEEN_MOVES)

				# switch to next player
				self.nextPlayer()

				# increase round game
				self.roundGame += 1

		# at this point one of the player has won, launch animation for robots
		self.endGame()

	def endGame(self):

		newMsg = Activity()

		winner = self.findWinner()
		if winner == 1 or winner == 2:
			newMsg.player = winner
			newMsg.state = "winner"
			publisher_activity.publish(newMsg)

	def checkIfVictory(self):

		if len(self.remainingCard) == 0:
			# finish the game
			self.gameFinished = True

	def stategyRobot(self):

		# first remove card in the list that cannot be played anymore (already success)
		# during the return, card1, card2 et state. State means : "confident" -> new two cards, "neutral" -> randomly returned first, new second, "random" new both

		# index = 0
		for cardReminded in self.listCardReminded:
			indexCardToDelete = cardReminded.getCard().getIndice()
			if cardReminded.getCard().getIndice() not in [c.getIndice() for c in self.remainingCard]:
				self.deleteRemindedCardAtIndex(indexCardToDelete)
			else:
				# get how old is the card reminded
				moveOld = self.roundGame - cardReminded.getRound()

				# the probability of forgetness is proportional to the moveOld
				factorPerf = 1
				probabilityForgetness = 1 - factorPerf/float(moveOld)
				if random.random() < probabilityForgetness and self.player == 2:
					self.deleteRemindedCardAtIndex(indexCardToDelete)

		# choose in function of the cards reminded by robot
		card1 = None
		card2 = None

		# first -> îf 2 cards same value in list
		for cardReminded in self.listCardReminded:
			valueCard = cardReminded.getCard().getValue()
			idCard = cardReminded.getCard().getIndice()
			for otherCardReminded in self.listCardReminded:
				if otherCardReminded.getCard().getValue() == valueCard and otherCardReminded.getCard().getIndice() != idCard:
					card1 = cardReminded.getCard()
					card2 = otherCardReminded.getCard()
					return card1, card2, "confident"

		# second if not 2 cards same value in list, choose first one randonly and second one depending on reminding list
		card1 = random.choice(self.remainingCard)
		for cardReminded in self.listCardReminded:
			if cardReminded.getCard().getValue() == card1.getValue() and cardReminded.getCard().getIndice() != card1.getIndice():
				card2 = cardReminded.getCard()
				return card1, card2, "neutral"


		# choose first card
		card1 = random.choice(self.remainingCard)
		card2 = random.choice(self.remainingCard)

		# two cards cannot have the same indice
		while(card1.getIndice() == card2.getIndice()):
			card2 = random.choice(self.remainingCard)

		return card1, card2, "random"

	def isPair(self, card1, card2):

		if card1.getValue() == card2.getValue():
			# remove card from list
			self.remainingCard.remove(card1)
			self.remainingCard.remove(card2)

			# increase success to this player by +1
			self.getPlayerbyId(self.player).increaseNbPoints()

			# check if victory
			self.checkIfVictory()

			# return success
			return True
		else:
			# return no success
			return False

	def callBackChildMove(self, data):
		
		# wait a moment
		rospy.sleep(TIME_BETWEEN_MOVES)

		# convert answer from topic to cards id
		childMove = data.data
		cardsId = childMove.split()

		# get cards object
		card1 = self.getCardByid(int(cardsId[0]))
		card2 = self.getCardByid(int(cardsId[1]))

		# check if success
		success = self.isPair(card1, card2)

		# log action
		self.listActionLog.append(ActionLog(card1, card2, 0, self.roundGame, success, datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

		# if success = false, important to keep that in mind for later move, if success not needed because robot won't play these cards anymway
		if self.gameFinished == False:
			self.listCardReminded.append(CardReminded(card1, self.roundGame))
			self.listCardReminded.append(CardReminded(card2, self.roundGame))

			# increase round game
			self.roundGame += 1

			# wait a moment
			rospy.sleep(TIME_BETWEEN_MOVES)

			# switch to next player
			self.nextPlayer()
		else:
			# at this point one of the player has won, launch animation for robots
			self.saveLogs()


	def callBackAnimationEnded(self, data):

		# if the animation id the return of a card
		if data.idAnimation == 0:
			# no animations are in progress anymore
			self.nbCardReturned += 1

		# log animation
		self.listAnimationLog.append(AnimationLog(data.idRobot, data.idAnimation, data.dateStart, data.dateEnd))

	def callBackGazeMatch(self, data):

		# get the robot id that the child watch
		robot = data.data

		# save that in GazeLog
		self.listGazeLog.append(GazeLog(robot, datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

	def saveLogs(self):

		# stop reccording audio + video
		publisher_end_record.publish("end")

		home = expanduser("~")
		date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
		path = home + "/outputMemory/" + KID_IDENTITY + "/"

		# create directory concerning the name of the user if does not exists
		try:
			os.stat(path)
		except:
			os.makedirs(path)


		if self.gameNumber == 3:
			self.gameNumber = 2

		# save action logs
		with open(path + "outputActions_" + str(self.gameNumber) + "_" + self.t0Experiment + ".txt", "w") as f:
			for actionLog in self.listActionLog:
				myStr = str(actionLog.roundGame) + "," + actionLog.date + "," + str(actionLog.player)
				myStr += "," + str(actionLog.card1.getIndice()) + "," + str(actionLog.card2.getIndice()) + "," + str(actionLog.success) + "\n"
				f.write(myStr)

			#f.write("winner is the player: " + str(self.findWinner()))
			f.close()

		# save gaze logs
		with open(path + "outputGaze_" + str(self.gameNumber) + "_" + self.t0Experiment + ".txt", "w") as f:
			for gazeLog in self.listGazeLog:
				myStr = gazeLog.date + "," + str(gazeLog.robotId) + "\n"
				f.write(myStr)

			f.close()

		# save animation logs
		with open(path + "outputAnimation_" + str(self.gameNumber) + "_" + self.t0Experiment + ".txt", "w") as f:
			for animationLog in self.listAnimationLog:
				myStr = animationLog.dateStart + "," + animationLog.dateEnd + ","
				myStr += str(animationLog.robotId) + "," + str(animationLog.animationId) + "\n"
				f.write(myStr)

			f.close()

	def findWinner(self):
		nbPoints = list()
		for player in xrange(self.nbPlayer):
			nbPoints.append(0)

		for actionLog in self.listActionLog:
			if actionLog.success == True:
				nbPoints[actionLog.player] += 1

		return nbPoints.index(max(nbPoints))

if __name__ == "__main__":

	# init node
	rospy.init_node("memory_game")

	# get topic params names
	TOPIC_ACTIVITY = rospy.get_param('~topic_activity')
	publisher_activity = rospy.Publisher(TOPIC_ACTIVITY, Activity, queue_size=10)

	TOPIC_PUBLISHER_TABLET = rospy.get_param('~topic_publisher_tablet')
	publisher_tablet = rospy.Publisher(TOPIC_PUBLISHER_TABLET, String, queue_size=10)

	TOPIC_PUBLISHER_END_RECORD = rospy.get_param('~topic_end_record')
	publisher_end_record = rospy.Publisher(TOPIC_PUBLISHER_END_RECORD, String, queue_size=10)

	# get the time to wait between player moves
	TIME_BETWEEN_MOVES = float(rospy.get_param('~time_between_moves'))
	# get the game number
	GAME_NUMBER = int(rospy.get_param('~game_number'))
	# get the kid identity
	KID_IDENTITY = rospy.get_param('~identity')


	# debug topic
	debug = rospy.Publisher("debug", String, queue_size=10)
	# give time for rospy to connect
	rospy.sleep(1)


	# the robots needs to introduce themself if it's the first game
	if GAME_NUMBER == 0:
		# first robot introduces himself
		newMsg = Activity()
		newMsg.player = 1
		newMsg.state = "introduction"
		publisher_activity.publish(newMsg)
		rospy.sleep(4)

		#second robot introduces himself
		newMsg.player = 2
		publisher_activity.publish(newMsg)

	# publish message saying that every robots are idle
	newMsg = Activity()
	newMsg.player = 1
	newMsg.state = "idle"
	publisher_activity.publish(newMsg)
	newMsg.player = 2
	publisher_activity.publish(newMsg)

	# launch the game
	game = MemoryGame(nbPlayer = 3, gameNumber = GAME_NUMBER)
	game.play()
	game.saveLogs()

	# publish message telling that the game is finished
	# rospy.sleep(15)
	# newMsg = Activity()
	# newMsg.player = 1
	# newMsg.state = "end"
	# publisher_activity.publish(newMsg)
	# newMsg.player = 2
	# publisher_activity.publish(newMsg)



	rospy.spin()





