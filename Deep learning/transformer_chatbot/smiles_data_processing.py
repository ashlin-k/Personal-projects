import os
# suppress warnings about dlls
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
from tensorflow.compat.v1 import layers
import numpy as np
import argparse
from chembl_webresource_client.new_client import new_client
from rdkit import Chem
from rdkit.Chem import Descriptors, QED
from chatbot_preprocess_data import get_args, get_tolkenized_dataset

# CHEMBL client: https://github.com/chembl/chembl_webresource_client
# RDKit tutorial: https://www.youtube.com/watch?v=9Z9XM9xamDU


def get_smiles_dataset(hparams : argparse.Namespace, molecule_name : str, similarity : float=70.0) -> \
    (tf.python.data.ops.dataset_ops.PrefetchDataset, 
	tfds.core.deprecated.text.subword_text_encoder.SubwordTextEncoder):

    # find molecules similar to morphine with 70% similarity
    molecule = new_client.molecule
    similarity = new_client.similarity

    try:
        chembl_id = molecule.search(molecule_name)[0]['molecule_chembl_id']
    except:
        print("Molecule could not be found")
        return []

    results = similarity.filter(chembl_id=chembl_id, similarity=similarity)

    # res is an array of query results. each query is a dictionary.
    # the SMILES representation of each query can be found from results as
    # results[i]['molecule_structures']['canonical_smiles']
    # 'molecule_structures' stores representations: ['canonical_smiles', 'molfile', 'standard_inchi', 'standard_inchi_key']

    # collect all smiles strings
    smiles_strings = []
    for query in results:
        smiles_strings.append(query['molecule_structures']['canonical_smiles'])

    # to get the Molecule object from SMILES using RDKit, rdkit.Chem.MolFromSmiles(<smiles_string[i]>) 

    smiles_tokenized, tokenizer = get_tolkenized_dataset(hparams, smiles_strings, smiles_strings)

    return smiles_tokenized, tokenizer


def get_descriptors(mol_smiles : str) -> (dict):

    # Lipinsky rule of 5
    # https://en.wikipedia.org/wiki/Lipinski%27s_rule_of_five

    mol = Chem.MolFromSmiles(mol_smiles)
    descriptors = {}

    qedProp = Chem.QED.properties(mol)
    descriptors['mw'] = qedProp.MW                  # molecular weight, should be less than 500 daltons
    descriptors['logp'] = qedProp.ALOGP             # lipophilicity, should be less than 5
    descriptors['rot_bonds'] = qedProp.ROTB         # should be less than 10
    descriptors['hbond_acceptors'] = qedProp.HBA    # should be less than 5
    descriptors['hbond_donors'] = qedProp.HBD       # should be less than 10

    return descriptors

def evaluate_molecule(smiles_string : str) -> (float):

    mol_descriptors = get_descriptors(smiles_string)

    # these standards of for orally active, similar drugs
    descriptor_standards = {
        'mw' : 500,
        'logp' : 5,
        'rot_bonds' : 10,
        'hbond_acceptors' : 5,
        'hbond_donors' : 10
    }

    score = 0.0
    for d in descriptor_standards:
        score += 1.0 if mol_descriptors[d] < descriptor_standards[d] else 0.0
    
    return score
