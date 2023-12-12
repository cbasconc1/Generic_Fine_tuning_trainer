#!/usr/bin/env python3
import pandas as pd
import numpy as np

import tensorflow as tf
from transformers import TFAutoModel, AutoTokenizer
import pickle



class BERTForClassification(tf.keras.Model):
    def __init__(self, bert_model, num_classes):
        super().__init__()
        self.bert = bert_model
        self.fc = tf.keras.layers.Dense(num_classes, activation='softmax')

    def call(self, inputs):
        x = self.bert(inputs)[1]
        return self.fc(x)

def model_build():
    
    with open('/home/cbas/ROS_Workspaces/nodes_prueba/src/repre_pack/src/pesos_levels.pkl', 'rb') as f:
        pesos_classifier = pickle.load(f)

    model2 = TFAutoModel.from_pretrained("dccuchile/bert-base-spanish-wwm-uncased")
    tokenizer = AutoTokenizer.from_pretrained("dccuchile/bert-base-spanish-wwm-uncased")

    classifier2 = BERTForClassification(model2, num_classes=5)
    classifier2.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=1e-5),
    loss=tf.keras.losses.SparseCategoricalCrossentropy(),
    metrics=['accuracy'] )

    input2 = {
    'input_ids': tf.constant([[1, 2, 3]]),
    'attention_mask': tf.constant([[1, 1, 1]]),
    'token_type_ids': tf.constant([[0, 0, 0]])
}
    classifier2(input2)
    classifier2.set_weights(pesos_classifier)


    return classifier2, tokenizer
#iniciar¿lizaar

def answer(instrucition, tokenizer, classifier2):

    inputs = tokenizer(instrucition, padding=True, truncation=True, return_tensors="tf")
    predicciones = classifier2(inputs)

# Obtener las clases predichas
    clases_predichas = np.argmax(predicciones, axis=1)


# Imprimir las clases predichas
    print("Nivel predicho:", clases_predichas)
    return(clases_predichas)

# Establecer los pesos en el modelo


# Tokenizar el texto

# Obtener la predicción usando el modelo

