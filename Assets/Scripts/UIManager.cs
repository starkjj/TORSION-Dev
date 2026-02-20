using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class UIManager : MonoBehaviour
{
    public Image rpmDial;
    public TextMeshProUGUI rpmText;
    
    public void SetVehicleProperties(float rpm, int gear)
    {
        rpmDial.fillAmount = rpm;
        rpmText.text = gear.ToString();
    }
    
}
