import numpy as np
from sklearn.model_selection import train_test_split

from models import amino_acids, read_sequences, ProPWMGenerator, LinderFitnessModel, LinderTrainer
from aa_encoding import *

def get_fitness_model(seqs : dict) -> (LinderFitnessModel, float):

    '''
    @param seqs:        A dictionary of format { sequence(str) : fitness(float) }
    @return model:      The trained LinderFitnessModel
    @return r2_test:    Returns the R2 score for the test samples.
    '''

    seq_length = len(list(seqs.keys())[0])
    aa_idx = { amino_acids[i] : i for i in range(len(amino_acids))}
    X, Y = aa_encoding(seqs, seq_length, aa_idx)
    Xcv, Xtest, Ycv, Ytest = train_test_split(X, Y, test_size=0.2, random_state=42)

    model_trainer = LinderTrainer()

    print("\nTraining fitness model\n")
    model_trainer.train_model(Xcv, Ycv, print_on=True)

    print("\nTesting fitness model\n")
    r2_test = model_trainer.test(Xtest, Ytest, print_on=True)

    fitness_model = model_trainer.get_trained_model()

    return fitness_model, r2_test

if __name__ == "__main__":
    
    seqs = read_sequences("DataSet for Assignment.xlsx - Sheet1.csv")
    seq_length = len(list(seqs.keys())[0])

    fitness_model, r2_fitness = get_fitness_model(seqs)
    sequence_generator = ProPWMGenerator(fitness_model, seq_length, amino_acids, wt_seq="VDGV", random_seed=5)

    ### TRAIN ###

    sequence_generator.train(seqs)
    print("\nFitness model R2 =", r2_fitness)

    ### TEST ###
    
    num_predicted = 10    
    print("\nPredicting top", num_predicted, "highest-fitness sequences\n")
    print("Seq\t|\tFitness")
    print("-------------------------------------")
    for i in range(num_predicted):
        novel_seq, fitness = sequence_generator.generate()
        print(novel_seq, "\t|\t", fitness)


