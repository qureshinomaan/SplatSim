import datetime
import types
from functools import wraps
import atexit

from joblib import register_compressor

registered_function = []
registered_code = {}

class TimeCounter() :

	def __init__(self, func) -> None:
		wraps(func)(self)
		registered_function.append(self)
		self.ncalls = 0
		self.total_time = 0

	def __call__(self, *args, **kwargs):

		self.ncalls += 1
		start_time = datetime.datetime.now()
		ret = self.__wrapped__(*args, **kwargs)
		end_time = datetime.datetime.now()
		self.total_time += (end_time-start_time).microseconds / 1000

		return ret

	def __get__(self, instance, cls):

		if instance is None:
			return self
		else:
			return types.MethodType(self, instance)

class TimeCounterSesion() :

	def __init__(self, name):

		self.name = name
		self.ncalls = 1
		self.total_time = 0

	def __enter__(self):

		self.start_time = datetime.datetime.now()
  	
	def __exit__(self, type, value, traceback):

		self.end_time = datetime.datetime.now()
		if self.name not in registered_code:
			registered_code[self.name] = self
		else :
			registered_code[self.name].ncalls += self.ncalls
			registered_code[self.name].total_time += (self.end_time-self.start_time).microseconds / 1000

@atexit.register
def print_profile() :

	print("Printing Profile:")

	for func in registered_function :

		print("\t{}(): ncalls={}, avgtime={}, totaltime={}".format(func.__wrapped__.__name__, func.ncalls, func.total_time/(func.ncalls+1e-8), func.total_time))
	
	for k,v in registered_code.items() :

		print("\t'{}': ncalls={}, avgtime={}, totaltime={}".format(k, v.ncalls, v.total_time/(v.ncalls+1e-8), v.total_time))
	
	print("End of Profile")