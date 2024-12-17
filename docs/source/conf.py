# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'ataraxis-transport-layer-mc'
# noinspection PyShadowingBuiltins
copyright = '2024, Ivan Kondratyev (Inkaros) & Sun Lab'
authors = ['Ivan Kondratyev (Inkaros)', 'Jasmine Si']
release = '1.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'breathe',           # To read doxygen-generated xml files (to parse C++ documentation).
    'sphinx_rtd_theme',  # To format the documentation HTML using ReadTheDocs format.
]

templates_path = ['_templates']
exclude_patterns = []

# Breathe configuration
breathe_projects = {"ataraxis-transport-layer-mc": "./doxygen/xml"}
breathe_default_project = "ataraxis-transport-layer-mc"

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'  # Directs sphinx to use RTD theme
