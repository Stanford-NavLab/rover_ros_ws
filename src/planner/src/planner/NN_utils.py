import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal

import params.params as params

class ActorCritic(nn.Module):
    """
    TODO
    Class for trained network
    """
    def __init__(self, num_inputs, num_outputs, hidden_sizes, std=0.0):
        super(ActorCritic, self).__init__()
        
        critic_modules = []
        critic_modules.append(nn.Linear(num_inputs, hidden_sizes[0]))
        critic_modules.append(nn.LeakyReLU(negative_slope=0.1))
        for i in range(1,len(hidden_sizes)):
            critic_modules.append(nn.Linear(hidden_sizes[i-1], hidden_sizes[i]))
            critic_modules.append(nn.LeakyReLU(negative_slope=0.1))
        
        actor_modules = []
        actor_modules.append(nn.Linear(num_inputs, hidden_sizes[0]))
        actor_modules.append(nn.LeakyReLU(negative_slope=0.1))
        for i in range(1,len(hidden_sizes)):
            actor_modules.append(nn.Linear(hidden_sizes[i-1], hidden_sizes[i]))
            actor_modules.append(nn.LeakyReLU(negative_slope=0.1))

        self.critic = nn.Sequential(
            *critic_modules,
            nn.Linear(hidden_sizes[-1], 1)
        )

        self.actor = nn.Sequential(
            *actor_modules,
            nn.Linear(hidden_sizes[-1], num_outputs),
            nn.Tanh()
        )

        self.log_std = nn.Parameter(torch.ones(1, num_outputs) * std)
                
    def forward(self, x):
        value = self.critic(x)
        mu    = self.actor(x)
        std   = self.log_std.exp().expand_as(mu)
        dist  = Normal(mu, std)
        return dist, value


def load_model(model_FileStr):
    """
    TODO
    Load trained network
    """
    
    #get number of inputs and outputs for model
    num_inputs  = 8; num_outputs = 2

    #load model to cpu
    use_cuda = False
    device   = torch.device("cuda" if use_cuda else "cpu")

    #load saved model and hyperparameters
    checkpoint = torch.load(model_FileStr, map_location=device)

    #load the learning params
    learning_params = checkpoint['learning_params']
    
    #load model, set params, and set to eval
    model = ActorCritic(num_inputs, num_outputs, learning_params['hidden_sizes']).to(device)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.train(mode=False)

    return model

def evaluate_model(model, x_nom):
    """
    TODO
    Get model output distribution mean and covariance
    """

    #get model device
    use_cuda = False
    device   = torch.device("cuda" if use_cuda else "cpu")

    #get input state for model
    state = torch.FloatTensor(np.concatenate((x_nom[:,[-1]],params.OBST_ARR),axis=0).T).to(device)

    #get model output distribution
    dist, _ = model(state)

    #get mean of output distribution
    action_mean = map_control_inputs(dist.loc.cpu().detach().numpy())

    #get covariance of output distribution
    if params.SAMPLE_METHOD == 1: action_cov = np.diag(dist.scale.cpu().detach().numpy()[0])
    elif params.SAMPLE_METHOD == 2: action_cov = params.SAMPLE_STD_DEV*np.identity(action_mean.shape[1])

    return [action_mean, action_cov]


def map_control_inputs(action_array):
    """
    TODO
    """
    for i in range(action_array.shape[0]):

        kw = action_array[i,0]; kv = action_array[i,1]
        kw_scaled = ((kw + 1.0)/2.0); kv_scaled = ((kv + 1.0)/2.0);  # between 0 and 1
        kw_mapped = kw_scaled*(params.KW_LIMS[1] - params.KW_LIMS[0]) + params.KW_LIMS[0]; #mapped to output range
        kv_mapped = kv_scaled*(params.KV_LIMS[1] - params.KV_LIMS[0]) + params.KV_LIMS[0]; #mapped to output range

        #get output array
        action_array[i,0] = np.clip(kw_mapped, params.KW_LIMS[0], params.KW_LIMS[1])
        action_array[i,1] = np.clip(kv_mapped, params.KV_LIMS[0], params.KV_LIMS[1])

    return action_array