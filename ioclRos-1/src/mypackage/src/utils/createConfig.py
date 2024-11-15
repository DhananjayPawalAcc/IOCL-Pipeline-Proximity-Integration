#!/bin/python3
import yaml
import os

def create(path):
    """
    DESCRIPTION:
        Loads config.yaml to access and tune various parameters used by pipeline
    ARGUMENTS:
        config_file (str): Path to config.yaml file
    RETURNS:
        config: dict
    """
    with open(path,'r') as file:
        configObj = yaml.safe_load(file)
    return configObj