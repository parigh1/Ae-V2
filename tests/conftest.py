# Makes `import config`, `import vision`, ... work no matter where pytest is run from.
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
