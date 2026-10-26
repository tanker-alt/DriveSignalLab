"""Best validation-loss epoch restoration, compatible with Keras 2.2."""
import math

class BestEpoch:
    def __init__(self):
        self.best=float('inf');self.epoch=None;self.weights=None;self.history=[]

    def observe(self, model, epoch, logs):
        value=float(logs.get('val_loss',float('nan')))
        self.history.append(dict(epoch=epoch,val_loss=value))
        if math.isfinite(value) and value<self.best:
            self.best=value;self.epoch=epoch
            self.weights=[w.copy() for w in model.get_weights()]

    def restore(self, model):
        if self.weights is None:raise RuntimeError('No finite validation loss; refuse checkpoint')
        model.set_weights(self.weights)
        return dict(best_epoch=self.epoch,best_val_loss=self.best,epochs=self.history)


def fit_best(agent, Callback, EarlyStopping):
    best=BestEpoch()
    class RestoreBest(Callback):
        def on_epoch_end(self, epoch, logs=None):best.observe(self.model,epoch,logs or {})
    history=agent.q_network.fit(agent.Xs,agent.Y,
        batch_size=min(agent.dic_agent_conf['BATCH_SIZE'],len(agent.Y)),
        epochs=agent.dic_agent_conf['EPOCHS'],shuffle=False,verbose=2,
        validation_split=.3,callbacks=[RestoreBest(),EarlyStopping(monitor='val_loss',patience=agent.dic_agent_conf['PATIENCE'],mode='min')])
    return best.restore(agent.q_network)


def target_checkpoint(update, lag):
    if update<0 or lag<1:raise ValueError('Invalid target update/lag')
    return 'initial' if update==0 else 'round_%03d'%max(update-lag,0)
