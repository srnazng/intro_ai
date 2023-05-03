from sklearn.neighbors import KNeighborsClassifier
import numpy as np
import util


class KNN:
  """
  Perceptron classifier.
  
  Note that the variable 'datum' in this code refers to a counter of features
  (not to a raw samples.Datum).
  """
  def __init__( self):
    self.type = "knn"

      
  def train( self, trainingData, trainingLabels, validationData, validationLabels ):
    """
    Training and validating the KNN classifier
    """
    self.features = list(set([ f for datum in trainingData for f in datum.keys() ]))

    self.X_train = np.zeros((len(trainingData), len(self.features)))
    self.y_train = np.array(trainingLabels)

    self.X_validation = np.zeros((len(validationData), len(self.features)))
    self.y_validation = np.array(validationLabels)

    for i, datum in enumerate(trainingData):
      for j, feature in enumerate(self.features):
        self.X_train[i, j] = datum[feature]

    for i, datum in enumerate(validationData):
      for j, feature in enumerate(self.features):
        self.X_validation[i, j] = datum[feature]

    best_accuracy = 0
    best_k = 0
    for k in range(1, 21):
      neigh = KNeighborsClassifier(n_neighbors = k)
      neigh.fit(self.X_train, self.y_train)

      predicted = neigh.predict(self.X_validation)
      accuracy = np.mean(predicted == self.y_validation)

      if accuracy > best_accuracy:
        best_k  = k
        best_accuracy = accuracy
    
    self.best_classifier = KNeighborsClassifier(n_neighbors = best_k)
    self.best_classifier.fit(self.X_train, self.y_train)
    # print(best_accuracy)


    
  def classify(self, data ):
    """
    Classifies each datum
    """
    self.X_test = np.zeros((len(data), len(self.features)))

    for i, datum in enumerate(data):
      for j, feature in enumerate(self.features):
        self.X_test[i, j] = datum[feature]

    return self.best_classifier.predict(self.X_test)

  



