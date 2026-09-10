# SimLingo-Base MORAI fine-tuning checkpoint

- Checkpoint: `best.pt`
- Training run: `simlingo_base_teacher_v1`
- Completed epochs: 30
- Selected best epoch: 26 (validation loss criterion)
- Validation loss: `0.704151`
- Validation ADE: `0.542723 m`
- Validation FDE @ 2 s: `1.135019 m`
- Training ADE at the selected epoch: `0.185099 m`
- Training FDE @ 2 s at the selected epoch: `0.365038 m`
- Batch size: 4
- Gradient accumulation: 7
- Learning rate: `3e-5`

The checkpoint contains the complete `model_state` and can be loaded with the MORAI SimLingo-Base code in `simlingo_base_morai`.
