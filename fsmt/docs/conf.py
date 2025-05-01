# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Free space motion tube'
copyright = '2025, Rômulo Rodrigues'
author = 'Rômulo Rodrigues'
release = 'v0.1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']

# Configuration file for the breathe documentation builder.

import os
import sys

sys.path.insert(0, os.path.abspath('../src'))

extensions = ["breathe"]

# Doxygen XML output directory
breathe_projects = {"My C Library": "doxygen_output/xml"}
breathe_default_project = "My C Library"

# HTML theme
html_theme = "alabaster"
