class Player():
	def __init__(self, idPlayer):
		self.id = idPlayer
		self.nbPoints = 0

	def getId(self):
		return self.id

	def increaseNbPoints(self):
		self.nbPoints += 1

	def getNbPoints(self):
		return self.nbPoints

