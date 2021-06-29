import os
# suppress warnings about dlls
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
from tensorflow.python.ops import math_ops
from tensorflow.python.ops import embedding_ops
from tensorflow import initializers
from tensorflow.python.keras.utils import tf_utils

# https://github.com/tensorflow/tensorflow/blob/master/tensorflow/python/keras/layers/embeddings.py

class EmbeddingLayer(tf.Module):

	def __init__(self, input_dim : int, output_dim : int, embeddings_initializer : str='uniform', \
		mask_zero : str=False, input_length : str=None):
		
		super(Embedding, self).__init__()
		self.input_dim = input_dim
		self.output_dim = output_dim
		self.embeddings_initializer = initializers.get(embeddings_initializer)
		self.mask_zero = mask_zero
		self.supports_masking = mask_zero
		self.input_length = input_length
		
		self.embeddings = self.add_weight(shape=(self.input_dim, self.output_dim), \
			initializer=self.embeddings_initializer, name='embeddings', experimental_autocast=False)
	
	def compute_mask(self, inputs : tf.Tensor, mask : tf.Tensor=None) -> (tf.Tensor):
		if not self.mask_zero:
			return None
		return math_ops.not_equal(inputs, 0)
	
	@tf_utils.shape_type_conversion
	def compute_output_shape(self, input_shape : tuple) -> (tuple):
    		
		if self.input_length is None:
			return input_shape + (self.output_dim,)
		else:
			# input_length can be tuple if input is 3D or higher
			if isinstance(self.input_length, (list, tuple)):
				in_lens = list(self.input_length)
			else:
				in_lens = [self.input_length]
			if len(in_lens) != len(input_shape) - 1:
				raise ValueError('"input_length" is %s, ' \
					'but received input has shape %s' % (str(
						self.input_length), str(input_shape)))
			else:
				for i, (s1, s2) in enumerate(zip(in_lens, input_shape[1:])):
					if s1 is not None and s2 is not None and s1 != s2:
						raise ValueError('"input_length" is %s, ' \
							'but received input has shape %s' % (str(self.input_length), str(input_shape)))
					elif s1 is None:
						in_lens[i] = s2
		
		return (input_shape[0],) + tuple(in_lens) + (self.output_dim,)
	
	def call(self, x : tf.Tensor):
    		
		dtype = x.dtype
		if dtype != tf.int32 and dtype != tf.int64:
			x = tf.cast(x, tf.int32)
		out = embedding_ops.embedding_lookup_v2(self.embeddings, x)
		if self._dtype_policy.compute_dtype != self._dtype_policy.variable_dtype:
            # Instead of casting the variable as in most layers, cast the output, as
            # this is mathematically equivalent but is faster.
			out = math_ops.cast(out, self._dtype_policy.compute_dtype)
		
		return out
