class CardReminded():
	def __init__(self, card, roundId):
		self.card = card
		self.round = roundId

	def getCard(self):
		return self.card

	def getRound(self):
		return self.round
