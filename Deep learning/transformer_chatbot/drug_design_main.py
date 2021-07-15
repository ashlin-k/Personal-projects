import os
# suppress warnings about dlls
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
from tensorflow.compat.v1 import layers
import numpy as np
import argparse
from transformer import Transformer
from chatbot_preprocess_data import get_args
from smiles_data_processing import *

# parameters taken from here: https://www.nature.com/articles/s41598-020-79682-4.pdf

VOCAB_SIZE = 71
NUM_LAYERS = 4
NUM_UNITS = 2*128
D_MODEL = 128
NUM_HEADS = 4
DROPOUT = 0.1  # not specifcied, perhaps not used
NUM_EPOCHS = 600000     # or try 20
BATCH_SIZE = 32
MAX_LENGTH = 4096 # ?

optimizer = tf.compat.v1.train.AdamOptimizer(learning_rate=0.001, beta1=0.9, beta2=0.98, epsilon=1e-9)

def loss_function(y_true : tf.Tensor, y_predict : tf.Tensor) -> (tf.Tensor):
    
    mask = tf.math.logical_not(tf.math.equal(y_true, 0))
    loss = tf.keras.losses.SparseCategoricalCrossentropy()(y_true, y_predict)
    mask = tf.cast(mask, dtype=loss.dtype)
    loss *= mask

    return tf.reduce_sum(loss)/tf.reduce_sum(mask)

def accuracy(y_true : tf.Tensor, y_pred : tf.Tensor) -> (tf.Tensor):

    assert y_true.shape[1] == y_pred.shape[1]
    accuracy = tf.metrics.SparseCategoricalAccuracy()(y_true, y_pred)
    return accuracy

def train_step(model : Transformer, x : tf.Tensor, dec_inputs : tf.Tensor, y_true : tf.Tensor) -> (tf.Tensor, tf.Tensor):
    
    with tf.GradientTape() as tape:
        y_predict = model(x, dec_inputs, training=True)
        loss = loss_function(y_true, y_predict)
        # l2_losses = model.get_l2_loss(loss.shape)
        # loss = tf.math.add(loss, 1e-5*l2_losses)

    # apply gradient
    dLoss = tape.gradient(loss, model.trainable_variables)
    optimizer.apply_gradients(zip(dLoss, model.trainable_variables))

    # loss
    training_loss = tf.reduce_mean(loss)

    return training_loss, y_predict

if __name__ == "__main__":

    hparams = get_args()

    smiles_dataset, tokenizer = get_smiles_dataset(hparams, 'morphine', similarity=70.0)

    model = Transformer(VOCAB_SIZE, NUM_LAYERS, NUM_UNITS, D_MODEL, NUM_HEADS, DROPOUT)
    training_losses, training_accuracy = {}, {}

    ### TRAIN ###

    for i in range(NUM_EPOCHS):

        y_true, y_predict = None, None

        for step, (x_batch_train, y_batch_train) in enumerate(smiles_dataset):

            y_true = y_batch_train
            training_loss, y_predict = train_step(model, x_batch_train['inputs'], \
                x_batch_train['dec_inputs'], y_batch_train)        
        
        training_losses[i] = training_loss
        training_accuracy[i] = accuracy(y_true, y_predict)

        print("Epoch", i+1, ", loss =", training_losses[i], ", accuracy =", training_accuracy[i])
    

    ### TEST ###

    evaluate(hparams, model, tokenizer)