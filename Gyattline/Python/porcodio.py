def riconosci_argento_process(frame_queue, argento_queue):
    # Carica il modello YOLO all'interno del processo
    model = YOLO("silver_classify_s.pt")
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            results = model.predict(frame, imgsz=128, conf=0.8, workers=4, verbose=False)
            print(results[0])
            confidence = results[0].probs.data[1].item()

            # Stampa il riepilogo tipo "0: 128x128 Silver 0.71, Line 0.29, 77.6ms"
            print("Confidenza argento",confidence)

            # Mostra il frame annotato
            annotated_frame = results[0].plot()
            cv2.imshow("YOLO - Inference", annotated_frame)
            argento_queue.put(confidence)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()