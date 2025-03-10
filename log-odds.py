# LOG ODDS
def log_odds (valor_atual, leitura, ocupacao=0.60, livre=0.40): ##OBS: Confiança a respeito do objeto
    if leitura == 1:
        incremento = math.log(ocupacao/livre)
    else:
        incremento = math.log(livre/ocupacao)
    novo_valor = valor_atual + incremento
    return max(-10,min(10, novo_valor))

