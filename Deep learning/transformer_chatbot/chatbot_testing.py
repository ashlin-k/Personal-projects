import os
# suppress warnings about dlls
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
import numpy as np
from chatbot_preprocess_data import preprocess_sentence

def inference(hparams, model, tokenizer, sentence):
    
    sentence = preprocess_sentence(sentence)

    sentence = tf.expand_dims(
        hparams.start_token + tokenizer.encode(sentence) + hparams.end_token,
        axis=0)

    output = tf.expand_dims(hparams.start_token, 0)

    for i in range(hparams.max_length):
        predictions = model(sentence, output, training=False)

        # select the last word from the seq_len dimension
        predictions = predictions[:, -1:, :]
        predicted_id = tf.cast(tf.argmax(predictions, axis=-1), tf.int32)

        # return the result if the predicted_id is equal to the end token
        if tf.equal(predicted_id, hparams.end_token[0]):
            break

        # concatenated the predicted_id to the output which is given to the decoder
        # as its input.
        output = tf.concat([output, predicted_id], axis=-1)

    return tf.squeeze(output, axis=0)


def predict(hparams, model, tokenizer, sentence):

    prediction = inference(hparams, model, tokenizer, sentence)

    predicted_sentence = tokenizer.decode([i for i in prediction if i > 0 and i < tokenizer.vocab_size])

    return predicted_sentence


def evaluate(hparams, model, tokenizer):
    print('\nEvaluate')
    sentence = 'where have you been?'
    output = predict(hparams, model, tokenizer, sentence)
    print('input: {}\noutput: {}'.format(sentence, output))

    sentence = "it's a trap!"
    output = predict(hparams, model, tokenizer, sentence)
    print('\ninput: {}\noutput: {}'.format(sentence, output))

    sentence = 'I am not crazy, my mother had me tested'
    for _ in range(5):
        output = predict(hparams, model, tokenizer, sentence)
        print('\ninput: {}\noutput: {}'.format(sentence, output))
        sentence = output