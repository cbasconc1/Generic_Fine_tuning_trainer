from transformers import AutoModelForSequenceClassification, AutoTokenizer

def model_build_devices():
    model_path = "/home/cbas/ROS_Workspaces/nodes_prueba/src/repre_pack/src/text-classification-model_local_trainer_V2"
    model = AutoModelForSequenceClassification.from_pretrained(model_path, num_labels=9)
    tokenizer = AutoTokenizer.from_pretrained("dccuchile/bert-base-spanish-wwm-uncased")
    print("modelito")
    return model, tokenizer

def answer(text, tokenizer, model):
    inputs = tokenizer(text, padding=True, truncation=True, max_length=512, return_tensors="pt")
    outputs = model(**inputs)
    probs = outputs[0].softmax(1)
    pred_label_idx = probs.argmax()
    pred_label = model.config.id2label[pred_label_idx.item()]
    pred_label =pred_label.replace("LABEL_", "")
    print("Nivel predicho:", pred_label)
    return(pred_label)


'''
text = "quisiera que porfavor si eres yan amable, abrieras la ventana izquierda, miamor"
model, tokenizer = model_build_devices()
label= answer(text,tokenizer,model)
print(label)'''
