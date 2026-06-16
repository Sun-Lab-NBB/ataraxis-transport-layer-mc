# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'ataraxis-transport-layer-mc'
copyright = '2026, Sun (NeuroAI) lab'
authors = ['Ivan Kondratyev', 'Jasmine Si']
release = '3.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'breathe',             # To read doxygen-generated xml files (to parse C++ documentation).
]

# Breathe configuration
breathe_projects = {"ataraxis-transport-layer-mc": "./doxygen/xml"}
breathe_default_project = "ataraxis-transport-layer-mc"

# -- Options for HTML output -------------------------------------------------
html_theme = 'furo'
