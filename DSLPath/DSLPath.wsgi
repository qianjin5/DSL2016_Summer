#!/usr/bin/python
import sys
import logging

activate_this = '/var/www/DSLPath/DSLPath/venv/bin/activate_this.py'
execfile (activate_this, dict(__file__ = activate_this))


logging.basicConfig(stream=sys.stderr)
sys.path.insert(0,"/var/www/DSLPath")

from DSLPath import app as application

