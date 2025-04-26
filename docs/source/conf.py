# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Cogar assignment'
copyright = '2025, Alessandro Mangili, Marco Lovecchio and Talha Rebbouh'
author = 'Alessandro Mangili, Marco Lovecchio and Talha Rebbouh'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_show_sourcelink = False

html_context = {
    "display_github": True,
    "github_user": "AlessandroMangili",
    "github_repo": "Assignment_Cogar_group_I",
    "github_version": "documentation",
    "conf_py_path": "/docs/source/",
}
