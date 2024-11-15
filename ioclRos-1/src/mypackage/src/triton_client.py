#!/bin/python3
import numpy as np
import tritonclient.http as httpclient
import tritonclient.grpc as grpcclient

class TritonModelSynchronous:
    """
    Class for interacting synchronously with a Triton Inference Server model.
    """
    def __init__(self, name, protocol, ip, port): # logger):
        """
        DESCRIPTION:
            Initialize the TritonModelSynchronous object.
        ARGUMENTS:
            name (str): The name of the model.
            protocol (str): The protocol to use for communication ('http' or 'grpc').
            ip (str): The IP address of the Triton Inference Server.
            port (int): The port of the Triton Inference Server.
            logger (logging.Logger): Logger object for logging events and logs for debugging.
        RETURNS:
        """
        self.name = name
        self.protocol = protocol
        #self.logger = logger
        if self.protocol == 'http':
            self.triton_client = httpclient.InferenceServerClient(url=f"{ip}:{port}")
        else:
            self.triton_client = grpcclient.InferenceServerClient(url=f'{ip}:{port}')

    def infer(self, image):
        """
        DESCRIPTION:
            Perform inference on the given image using the configured Triton Inference Server.
        ARGUMENTS:
            image (numpy.ndarray): The input image for inference.
        RETURNS:
            results: list
        """
        if self.protocol == 'http':
            model_config = self.triton_client.get_model_config(self.name)['config']
        else:
            model_config = self.triton_client.get_model_config(self.name, as_json=True)['config']
        image = np.expand_dims(image, axis=0)
        if self.protocol == 'http':
            inputs = httpclient.InferInput(model_config['input'][0]['name'], image.shape, datatype=model_config['input'][0]['data_type'].split('TYPE_')[-1])
            inputs.set_data_from_numpy(image, binary_data=True)
            outputs = [httpclient.InferRequestedOutput(model_config['output'][i]['name'], binary_data=True) for i, _ in enumerate(model_config['output'])]
        else:
            inputs = grpcclient.InferInput(model_config['input'][0]['name'], image.shape, datatype=model_config['input'][0]['data_type'].split('TYPE_')[-1])
            inputs.set_data_from_numpy(image)
            outputs = [grpcclient.InferRequestedOutput(model_config['output'][i]['name']) for i, output in enumerate(model_config['output'])]
        results = self.triton_client.infer(model_name=self.name, inputs=[inputs], outputs=outputs)
        results = [results.as_numpy(output['name']) for output in model_config['output']]
        return results
