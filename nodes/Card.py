class Card():
	def __init__(self, indice, value):
		self.indice = indice
		self.value = value

	def getIndice(self):
		return self.indice

	def setIndice(self, newIndice):
		self.indice = newIndice

	def getValue(self):
		return self.value

	def setValue(self, newValue):
		self.value = newValue
